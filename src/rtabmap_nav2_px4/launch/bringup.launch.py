#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use /clock if running in Gazebo/IGN'
    )

    run_offboard_arg = DeclareLaunchArgument(
        'run_offboard',
        default_value='false',
        description='Launch px4_offboard/offboard_control (Offboard heartbeat)',
    )

    start_offboard_arg = DeclareLaunchArgument(
        'start_offboard',
        default_value='false',
        description='Send Offboard heartbeats/mode from velocity_transform',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    run_offboard = LaunchConfiguration('run_offboard')
    start_offboard = LaunchConfiguration('start_offboard')

    rtabmap_pkg_share = FindPackageShare('rtabmap_nav2_px4')
    px4_off_pkg_share = FindPackageShare('px4_offboard')

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'spawn_robot.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_offboard': start_offboard,
        }.items()
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'rtabmap.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rtabmap_pkg_share, 'launch', 'navigation.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,       # nav2 측 인자는 다를 수 있음
        }.items()
    )

    offboard_vel_ctrl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([px4_off_pkg_share,
                                  'offboard_position_control.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(run_offboard),
    )


    # ─────────────────────────────────────────────
    # 4) LaunchDescription 반환
    # ─────────────────────────────────────────────
    return LaunchDescription([
        use_sim_time_arg,
        run_offboard_arg,
        start_offboard_arg,
        spawn_robot,
        rtabmap,
        navigation,
        # if you want to use package in SLAM mode, comment out the line above
        # else, uncomment the line above
        offboard_vel_ctrl,
    ])
