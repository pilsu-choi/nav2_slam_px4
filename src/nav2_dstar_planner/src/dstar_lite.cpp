#include "nav2_dstar_planner/dstar_lite.hpp"
#include <algorithm>
#include <cmath>

namespace nav2_dstar_planner
{

DStarLite::DStarLite(int width, int height)
: width_(width), height_(height), start_x_(0), start_y_(0), goal_x_(0), goal_y_(0),
  last_x_(0), last_y_(0)
{
  // Initialize priority queue with comparator after member variables are constructed
  open_list_ = std::priority_queue<Node, std::vector<Node>, NodeComparator>(getComparator());
}

void DStarLite::initialize(int start_x, int start_y, int goal_x, int goal_y)
{
  start_x_ = start_x;
  start_y_ = start_y;
  goal_x_ = goal_x;
  goal_y_ = goal_y;
  last_x_ = start_x;
  last_y_ = start_y;
  
  // Clear previous data
  reset();
  
  // Initialize goal
  Node goal_node(goal_x, goal_y);
  rhs_values_[goal_node] = 0.0;
  g_values_[goal_node] = INFINITY;
  
  // Calculate heuristic for all nodes (lazy: calculate on demand)
  // We'll calculate heuristics lazily when needed
  
  // Add goal to open list
  open_list_ = std::priority_queue<Node, std::vector<Node>, NodeComparator>(getComparator());
  open_list_.push(goal_node);
  open_set_.insert(goal_node);
}

void DStarLite::reset()
{
  g_values_.clear();
  rhs_values_.clear();
  h_values_.clear();
  obstacles_.clear();
  open_list_ = std::priority_queue<Node, std::vector<Node>, NodeComparator>(getComparator());
  open_set_.clear();
}

bool DStarLite::needsResize(int width, int height) const
{
  return width_ != width || height_ != height;
}

void DStarLite::resize(int width, int height)
{
  width_ = width;
  height_ = height;
  reset();
}

std::vector<std::pair<int, int>> DStarLite::computePath()
{
  computeShortestPath();
  
  std::vector<std::pair<int, int>> path;
  Node start_node = getNode(start_x_, start_y_);
  
  // Check if start has valid g value (path exists)
  if (g_values_.find(start_node) == g_values_.end() || 
      g_values_[start_node] >= INFINITY) {
    return path;  // No path found
  }
  
  Node current = start_node;
  path.push_back({current.x, current.y});
  
  while (!(current.x == goal_x_ && current.y == goal_y_)) {
    std::vector<Node> neighbors = getNeighbors(current);
    Node best_neighbor = current;
    double best_cost = INFINITY;
    
    for (const auto & neighbor : neighbors) {
      auto g_it = g_values_.find(neighbor);
      if (g_it != g_values_.end() && g_it->second < INFINITY) {
        double cost = g_it->second + calculateCost(current, neighbor);
        if (cost < best_cost) {
          best_cost = cost;
          best_neighbor = neighbor;
        }
      }
    }
    
    if (best_neighbor.x == current.x && best_neighbor.y == current.y) {
      // No valid path found
      break;
    }
    
    current = best_neighbor;
    path.push_back({current.x, current.y});
    
    // Prevent infinite loops
    if (path.size() > static_cast<size_t>(width_ * height_)) {
      break;
    }
  }
  
  return path;
}

void DStarLite::updateCost(int x, int y, double /* cost */)
{
  Node node(x, y);
  // Update cost in the costmap
  // This would typically update the underlying costmap
}

void DStarLite::updateObstacle(int x, int y, bool is_obstacle)
{
  Node node(x, y);
  if (is_obstacle) {
    obstacles_.insert(node);
  } else {
    obstacles_.erase(node);
  }
  
  // Update affected vertices
  std::vector<Node> neighbors = getNeighbors(node);
  for (const auto & neighbor : neighbors) {
    updateVertex(neighbor);
  }
  updateVertex(node);
}

void DStarLite::setStart(int x, int y)
{
  start_x_ = x;
  start_y_ = y;
}

void DStarLite::setGoal(int x, int y)
{
  // Only clear heuristics if goal actually changed
  if (goal_x_ != x || goal_y_ != y) {
    goal_x_ = x;
    goal_y_ = y;
    // Clear heuristics - they'll be recalculated lazily
    h_values_.clear();
  }
}

double DStarLite::calculateHeuristic(int x1, int y1, int x2, int y2)
{
  return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

double DStarLite::calculateKey(const Node & node)
{
  double g_val = (g_values_.find(node) != g_values_.end()) ? g_values_[node] : INFINITY;
  double rhs_val = (rhs_values_.find(node) != rhs_values_.end()) ? rhs_values_[node] : INFINITY;
  
  // Lazy heuristic calculation
  double h_val = 0.0;
  auto h_it = h_values_.find(node);
  if (h_it == h_values_.end()) {
    h_val = calculateHeuristic(node.x, node.y, goal_x_, goal_y_);
    h_values_[node] = h_val;
  } else {
    h_val = h_it->second;
  }
  
  return std::min(g_val, rhs_val) + h_val;
}

std::vector<Node> DStarLite::getNeighbors(const Node & node)
{
  std::vector<Node> neighbors;
  
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      
      int nx = node.x + dx;
      int ny = node.y + dy;
      
      if (isValid(nx, ny)) {
        Node neighbor(nx, ny);
        if (obstacles_.find(neighbor) == obstacles_.end()) {
          neighbors.push_back(neighbor);
        }
      }
    }
  }
  
  return neighbors;
}

double DStarLite::calculateCost(const Node & from, const Node & to)
{
  double dx = to.x - from.x;
  double dy = to.y - from.y;
  return std::sqrt(dx * dx + dy * dy);
}

void DStarLite::updateVertex(const Node & node)
{
  if (node.x == goal_x_ && node.y == goal_y_) {
    rhs_values_[node] = 0.0;
    return;
  }
  
  std::vector<Node> neighbors = getNeighbors(node);
  double min_rhs = INFINITY;
  
  for (const auto & neighbor : neighbors) {
    auto g_it = g_values_.find(neighbor);
    if (g_it != g_values_.end() && g_it->second < INFINITY) {
      double cost = g_it->second + calculateCost(node, neighbor);
      min_rhs = std::min(min_rhs, cost);
    }
  }
  
  rhs_values_[node] = min_rhs;
  
  // Remove from open list if present
  if (open_set_.find(node) != open_set_.end()) {
    open_set_.erase(node);
  }
  
  // Add to open list if inconsistent
  double g_val = (g_values_.find(node) != g_values_.end()) ? g_values_[node] : INFINITY;
  if (std::abs(g_val - rhs_values_[node]) > 1e-9) {
    open_list_.push(node);
    open_set_.insert(node);
  }
}

void DStarLite::computeShortestPath()
{
  Node start_node = getNode(start_x_, start_y_);
  int max_iterations = width_ * height_ * 10;  // Prevent infinite loops
  int iterations = 0;
  
  while (!open_list_.empty() && iterations < max_iterations) {
    iterations++;
    
    // Check if start is consistent
    double start_g = (g_values_.find(start_node) != g_values_.end()) ? g_values_[start_node] : INFINITY;
    double start_rhs = (rhs_values_.find(start_node) != rhs_values_.end()) ? rhs_values_[start_node] : INFINITY;
    
    if (std::abs(start_g - start_rhs) < 1e-9) {
      // Check if top node has key >= start key
      if (!open_list_.empty()) {
        Node top = open_list_.top();
        double top_key = calculateKey(top);
        double start_key = calculateKey(start_node);
        if (top_key >= start_key - 1e-9) {
          break;  // Start is consistent and top key is not better
        }
      } else {
        break;
      }
    }
    
    Node current = open_list_.top();
    open_list_.pop();
    
    // Check if node is still in open set (might have been updated)
    if (open_set_.find(current) == open_set_.end()) {
      continue;
    }
    open_set_.erase(current);
    
    double g_val = (g_values_.find(current) != g_values_.end()) ? g_values_[current] : INFINITY;
    double rhs_val = (rhs_values_.find(current) != rhs_values_.end()) ? rhs_values_[current] : INFINITY;
    
    if (g_val > rhs_val) {
      g_values_[current] = rhs_val;
      
      std::vector<Node> neighbors = getNeighbors(current);
      for (const auto & neighbor : neighbors) {
        updateVertex(neighbor);
      }
    } else {
      g_values_[current] = INFINITY;
      updateVertex(current);
      
      std::vector<Node> neighbors = getNeighbors(current);
      for (const auto & neighbor : neighbors) {
        updateVertex(neighbor);
      }
    }
  }
}

bool DStarLite::isValid(int x, int y)
{
  return x >= 0 && x < width_ && y >= 0 && y < height_;
}

Node DStarLite::getNode(int x, int y)
{
  return Node(x, y);
}

}  // namespace nav2_dstar_planner
