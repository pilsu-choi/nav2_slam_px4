#include "nav2_dstar_planner/dstar_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace nav2_dstar_planner
{

DStarPlanner::DStarPlanner()
: costmap_(nullptr), tolerance_(0.5), use_astar_(false), allow_unknown_(true),
  last_goal_mx_(-1), last_goal_my_(-1), last_costmap_width_(0), last_costmap_height_(0),
  is_initialized_(false)
{
}

void DStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();
  
  auto node = node_.lock();
  declareParameters();
  getParameters();
  
  // Initialize DStarLite with current costmap size
  unsigned int width = costmap_->getSizeInCellsX();
  unsigned int height = costmap_->getSizeInCellsY();
  dstar_lite_ = std::make_unique<DStarLite>(width, height);
  last_costmap_width_ = width;
  last_costmap_height_ = height;
  is_initialized_ = false;
  
  RCLCPP_INFO(node->get_logger(), "Configuring DStarPlanner with costmap size %ux%u", width, height);
}

void DStarPlanner::cleanup()
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Cleaning up DStarPlanner");
  dstar_lite_.reset();
}

void DStarPlanner::activate()
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Activating DStarPlanner");
}

void DStarPlanner::deactivate()
{
  RCLCPP_INFO(node_.lock()->get_logger(), "Deactivating DStarPlanner");
}

nav_msgs::msg::Path DStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path;
  path.header.stamp = node_.lock()->now();
  path.header.frame_id = costmap_ros_->getGlobalFrameID();
  
  auto node = node_.lock();
  
  // Convert start and goal to map coordinates
  int start_mx, start_my, goal_mx, goal_my;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my)) {
    RCLCPP_WARN(node->get_logger(), "Start point is outside the costmap");
    return path;
  }
  
  if (!worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)) {
    RCLCPP_WARN(node->get_logger(), "Goal point is outside the costmap");
    return path;
  }
  
  // Check if start and goal are valid
  if (!isCellFree(start_mx, start_my) && !allow_unknown_) {
    RCLCPP_WARN(node->get_logger(), "Start point is occupied");
    return path;
  }
  
  if (!isCellFree(goal_mx, goal_my) && !allow_unknown_) {
    RCLCPP_WARN(node->get_logger(), "Goal point is occupied");
    return path;
  }
  
  // Check if costmap size changed
  unsigned int current_width = costmap_->getSizeInCellsX();
  unsigned int current_height = costmap_->getSizeInCellsY();
  if (dstar_lite_->needsResize(current_width, current_height)) {
    dstar_lite_->resize(current_width, current_height);
    last_costmap_width_ = current_width;
    last_costmap_height_ = current_height;
    is_initialized_ = false;
  }
  
  // Check if goal changed or first initialization
  bool goal_changed = (goal_mx != last_goal_mx_ || goal_my != last_goal_my_);
  bool needs_full_init = !is_initialized_ || goal_changed;
  
  if (needs_full_init) {
    // Full initialization with new goal
    dstar_lite_->initialize(start_mx, start_my, goal_mx, goal_my);
    last_goal_mx_ = goal_mx;
    last_goal_my_ = goal_my;
    is_initialized_ = true;
    
    // Update obstacles from costmap (only on first init or goal change)
    // This is still expensive but necessary for correctness
    for (unsigned int mx = 0; mx < current_width; ++mx) {
      for (unsigned int my = 0; my < current_height; ++my) {
        unsigned char cost = costmap_->getCost(mx, my);
        if (cost == nav2_costmap_2d::LETHAL_OBSTACLE) {
          dstar_lite_->updateObstacle(mx, my, true);
        }
      }
    }
  } else {
    // Just update start position
    dstar_lite_->setStart(start_mx, start_my);
  }
  
  // Compute path
  std::vector<std::pair<int, int>> path_cells = dstar_lite_->computePath();
  
  if (path_cells.empty()) {
    RCLCPP_WARN(node->get_logger(), "No path found");
    return path;
  }
  
  // Convert path to world coordinates
  path.poses = createPath(path_cells);
  
  RCLCPP_INFO(node->get_logger(), "Found path with %zu points", path.poses.size());
  
  return path;
}

void DStarPlanner::declareParameters()
{
  auto node = node_.lock();
  node->declare_parameter(name_ + ".tolerance", 0.5);
  node->declare_parameter(name_ + ".use_astar", false);
  node->declare_parameter(name_ + ".allow_unknown", true);
}

void DStarPlanner::getParameters()
{
  auto node = node_.lock();
  node->get_parameter(name_ + ".tolerance", tolerance_);
  node->get_parameter(name_ + ".use_astar", use_astar_);
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
}

bool DStarPlanner::worldToMap(double wx, double wy, int & mx, int & my)
{
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  
  mx = static_cast<int>((wx - origin_x) / resolution);
  my = static_cast<int>((wy - origin_y) / resolution);
  
  return mx >= 0 && mx < static_cast<int>(costmap_->getSizeInCellsX()) &&
         my >= 0 && my < static_cast<int>(costmap_->getSizeInCellsY());
}

void DStarPlanner::mapToWorld(int mx, int my, double & wx, double & wy)
{
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  
  wx = origin_x + (mx + 0.5) * resolution;
  wy = origin_y + (my + 0.5) * resolution;
}

bool DStarPlanner::isCellFree(int mx, int my)
{
  unsigned char cost = costmap_->getCost(mx, my);
  return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

std::vector<geometry_msgs::msg::PoseStamped> DStarPlanner::createPath(
  const std::vector<std::pair<int, int>> & path)
{
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  
  for (const auto & cell : path) {
    double wx, wy;
    mapToWorld(cell.first, cell.second, wx, wy);
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_.lock()->now();
    pose.header.frame_id = costmap_ros_->getGlobalFrameID();
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.w = 1.0;
    
    poses.push_back(pose);
  }
  
  return poses;
}

}  // namespace nav2_dstar_planner

PLUGINLIB_EXPORT_CLASS(nav2_dstar_planner::DStarPlanner, nav2_core::GlobalPlanner)

