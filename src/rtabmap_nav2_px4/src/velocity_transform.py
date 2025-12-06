#!/usr/bin/env python3
"""
Bridge ROS2 geometry_msgs/Twist (base_link-ENU, FLU) → PX4 TrajectorySetpoint (body-NED, FRD).
Both world-frame (ENU↔NED) and body-frame (FLU↔FRD) conversions are done explicitly so that
all sign flips are easy to audit.

Layout
------
• class **TwistToTrajectoryNode - main rclpy.Node
    ├─ _ros_to_frd()   - FLU → FRD (body frame)
    ├─ _frd_to_ned_world() - rotate body-FRD → world-NED using current attitude
    ├─ _ros_yaw_to_px4()   - angular.z conversion (FLU→FRD, ENU→NED)
    └─ timers / subs / pubs as usual
"""

from __future__ import annotations
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from geometry_msgs.msg import Twist
from px4_msgs.msg import (
    TrajectorySetpoint,
    VehicleAttitude,
    VehicleLocalPosition,
    OffboardControlMode,
    VehicleCommand,
    VehicleCommandAck,
    VehicleStatus,
)

# -----------------------------------------------------------------------------
# Helper – quaternion → yaw (NED convention, yaw about Down ‑Z)
# PX4 attitude msg stores q = (w,x,y,z) already in **FRD‑NED** frame.
# -----------------------------------------------------------------------------

def quat_to_yaw_ned(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class TwistToTrajectoryNode(Node):
    """Subscribe /cmd_vel and publish TrajectorySetpoint."""

    def __init__(self):
        super().__init__("twist_to_traj_node")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_ack = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,  # ack publisher is volatile
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # allow disabling offboard enforcement so QGC commands (RTL/Land) work
        self._enable_offboard_param = (
            self.declare_parameter("enable_offboard", True)
            .get_parameter_value()
            .bool_value
        )
        # live gate: opened only when PX4 is in Offboard and no RTL/Land requested
        self._offboard_status_ok = True

        # internal state -----------------------------------------------------------------
        self._body_yaw_ned = 0.0  # current yaw angle of vehicle in NED (rad)
        self._cmd_lin_flu = np.zeros(3)  # latest linear velocity in FLU/ENU (m/s)
        self._cmd_yaw_flu = 0.0  # latest angular z in FLU/ENU (rad/s)
        self._last_mode_cmd_ns = 0  # throttle Offboard mode commands

        # subscribers --------------------------------------------------------------------
        self.create_subscription(Twist, "/cmd_vel", self._twist_cb, 10)
        self.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_cb,
            qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self._local_cb,
            qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._status_cb,
            qos,
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack",
            self._cmd_ack_cb,
            qos_ack,
        )

        # publisher ----------------------------------------------------------------------
        self._pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self._pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self._pub_mode_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # timer --------------------------------------------------------------------------
        self.create_timer(0.02, self._publish_setpoint)  # 50 Hz

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------
    def _twist_cb(self, msg: Twist):
        """Store incoming cmd_vel (base_link, FLU, ENU)."""
        self._cmd_lin_flu = np.array(
            [msg.linear.x, msg.linear.y, msg.linear.z], dtype=float
        )
        self._cmd_yaw_flu = msg.angular.z

    def _attitude_cb(self, msg: VehicleAttitude):
        self._body_yaw_ned = quat_to_yaw_ned(msg.q)

    def _local_cb(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg

    def _status_cb(self, msg: VehicleStatus):
        # allow Offboard publish only while PX4 reports Offboard nav_state
        self._offboard_status_ok = (
            msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def _cmd_ack_cb(self, msg: VehicleCommandAck):
        # if QGC/FCU acknowledged RTL or Land, stop Offboard enforcement
        if msg.command in (
            VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH,
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
        ):
            self._offboard_status_ok = False

    # ---------------------------------------------------------------------
    # Conversion helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def _ros_to_frd(v_flu: np.ndarray) -> np.ndarray:
        """FLU → FRD (body frame sign flips)."""
        #   FLU (x fwd, y left, z up)  →  FRD (x fwd, y right, z down)
        x, y, z = v_flu
        return np.array([x, -y, -z])

    def _frd_to_ned_world(self, v_frd: np.ndarray) -> np.ndarray:
        """Rotate body‑FRD velocity into world‑NED using current yaw."""
        cy = math.cos(self._body_yaw_ned)
        sy = math.sin(self._body_yaw_ned)
        rot = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        return rot @ v_frd

    @staticmethod
    def _ros_yaw_to_px4(yaw_rate_flu: float) -> float:
        """Angular z : +CCW about +Z (FLU) equals +CCW about −Z (FRD).
        The numeric sign is therefore preserved.
        """
        return -yaw_rate_flu

    # ---------------------------------------------------------------------
    # Main loop – build & publish TrajectorySetpoint
    # ---------------------------------------------------------------------
    def _publish_setpoint(self):
        if not (self._enable_offboard_param and self._offboard_status_ok):
            # Offboard is disabled: don't send heartbeat, mode commands, or setpoints
            return

        now_ns = self.get_clock().now().nanoseconds
        now_us = int(now_ns / 1000)

        # Offboard heartbeat (velocity mode)
        hb = OffboardControlMode()
        hb.timestamp = now_us
        hb.position = False
        hb.velocity = True
        hb.acceleration = False
        hb.attitude = False
        hb.body_rate = False
        hb.thrust_and_torque = False
        hb.direct_actuator = False
        self._pub_offboard.publish(hb)

        # Periodic Offboard mode command (2 Hz) to keep PX4 in Offboard
        if now_ns - self._last_mode_cmd_ns > 500_000_000:
            cmd = VehicleCommand()
            cmd.timestamp = now_us
            cmd.param1 = 1.0  # custom main mode
            cmd.param2 = 6.0  # main mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD
            cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
            cmd.target_system = 1
            cmd.target_component = 1
            cmd.source_system = 1
            cmd.source_component = 1
            cmd.from_external = True
            self._pub_mode_cmd.publish(cmd)
            self._last_mode_cmd_ns = now_ns

        # 1) Body‑frame conversion --------------------------------------------------------
        v_frd = self._ros_to_frd(self._cmd_lin_flu)

        # 2) World‑frame rotation ---------------------------------------------------------
        v_ned = self._frd_to_ned_world(v_frd)

        # 3) Pack PX4 TrajectorySetpoint --------------------------------------------------
        msg = TrajectorySetpoint()
        msg.timestamp = now_us
        msg.position[:] = [float('nan'), float('nan'), float('nan')]
        msg.acceleration[:] = [float("nan")] * 3
        msg.velocity[:] = v_ned.astype(float)
        msg.yaw = float("nan")  # keep current heading
        msg.yawspeed = self._ros_yaw_to_px4(self._cmd_yaw_flu)

        self._pub.publish(msg)


# --------------------------------------------------------------------------------------
#   main
# --------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
