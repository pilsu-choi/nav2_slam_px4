#ifndef NAV2_DSTAR_PLANNER__DSTAR_PLANNER_HPP_
#define NAV2_DSTAR_PLANNER__DSTAR_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_dstar_planner/dstar_lite.hpp"

namespace nav2_dstar_planner
{

class DStarPlanner : public nav2_core::GlobalPlanner
{
public:
  DStarPlanner();
  ~DStarPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  
  std::unique_ptr<DStarLite> dstar_lite_;
  
  double tolerance_;
  bool use_astar_;
  bool allow_unknown_;
  
  // Caching for optimization
  int last_goal_mx_, last_goal_my_;
  unsigned int last_costmap_width_, last_costmap_height_;
  bool is_initialized_;
  
  // Parameters
  void declareParameters();
  void getParameters();
  
  // Utility functions
  bool worldToMap(double wx, double wy, int & mx, int & my);
  void mapToWorld(int mx, int my, double & wx, double & wy);
  bool isCellFree(int mx, int my);
  std::vector<geometry_msgs::msg::PoseStamped> createPath(
    const std::vector<std::pair<int, int>> & path);
};

}  // namespace nav2_dstar_planner

#endif  // NAV2_DSTAR_PLANNER__DSTAR_PLANNER_HPP_
