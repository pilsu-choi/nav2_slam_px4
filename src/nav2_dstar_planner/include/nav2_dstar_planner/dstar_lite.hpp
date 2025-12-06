#ifndef NAV2_DSTAR_PLANNER__DSTAR_LITE_HPP_
#define NAV2_DSTAR_PLANNER__DSTAR_LITE_HPP_

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <algorithm>
#include <memory>
#include <limits>

#ifndef INFINITY
#define INFINITY std::numeric_limits<double>::infinity()
#endif

namespace nav2_dstar_planner
{

struct Node
{
  int x, y;
  double g, rhs;
  double h;
  bool is_obstacle;
  
  Node(int x = 0, int y = 0) : x(x), y(y), g(INFINITY), rhs(INFINITY), h(0.0), is_obstacle(false) {}
  
  bool operator==(const Node& other) const
  {
    return x == other.x && y == other.y;
  }
};

struct NodeHash
{
  std::size_t operator()(const Node& node) const
  {
    return std::hash<int>()(node.x) ^ (std::hash<int>()(node.y) << 1);
  }
};

struct NodeComparator
{
  std::unordered_map<Node, double, NodeHash>* g_values_;
  std::unordered_map<Node, double, NodeHash>* rhs_values_;
  std::unordered_map<Node, double, NodeHash>* h_values_;
  
  NodeComparator()
    : g_values_(nullptr), rhs_values_(nullptr), h_values_(nullptr) {}
  
  NodeComparator(
    std::unordered_map<Node, double, NodeHash>* g,
    std::unordered_map<Node, double, NodeHash>* rhs,
    std::unordered_map<Node, double, NodeHash>* h)
    : g_values_(g), rhs_values_(rhs), h_values_(h) {}
  
  bool operator()(const Node& a, const Node& b) const
  {
    if (!g_values_ || !rhs_values_ || !h_values_) {
      return false;  // Default comparison if not initialized
    }
    
    double g_a = (g_values_->find(a) != g_values_->end()) ? (*g_values_)[a] : INFINITY;
    double rhs_a = (rhs_values_->find(a) != rhs_values_->end()) ? (*rhs_values_)[a] : INFINITY;
    double h_a = (h_values_->find(a) != h_values_->end()) ? (*h_values_)[a] : 0.0;
    double key_a = std::min(g_a, rhs_a) + h_a;
    
    double g_b = (g_values_->find(b) != g_values_->end()) ? (*g_values_)[b] : INFINITY;
    double rhs_b = (rhs_values_->find(b) != rhs_values_->end()) ? (*rhs_values_)[b] : INFINITY;
    double h_b = (h_values_->find(b) != h_values_->end()) ? (*h_values_)[b] : 0.0;
    double key_b = std::min(g_b, rhs_b) + h_b;
    
    if (std::abs(key_a - key_b) < 1e-9) {
      return std::min(g_a, rhs_a) > std::min(g_b, rhs_b);
    }
    return key_a > key_b;
  }
};

class DStarLite
{
public:
  DStarLite(int width, int height);
  ~DStarLite() = default;
  
  void initialize(int start_x, int start_y, int goal_x, int goal_y);
  void reset();  // Clear all data but keep dimensions
  std::vector<std::pair<int, int>> computePath();
  void updateCost(int x, int y, double cost);
  void updateObstacle(int x, int y, bool is_obstacle);
  void setStart(int x, int y);
  void setGoal(int x, int y);
  bool needsResize(int width, int height) const;
  void resize(int width, int height);
  
private:
  int width_, height_;
  int start_x_, start_y_;
  int goal_x_, goal_y_;
  int last_x_, last_y_;
  
  std::unordered_map<Node, double, NodeHash> g_values_;
  std::unordered_map<Node, double, NodeHash> rhs_values_;
  std::unordered_map<Node, double, NodeHash> h_values_;
  std::unordered_set<Node, NodeHash> obstacles_;
  
  std::priority_queue<Node, std::vector<Node>, NodeComparator> open_list_;
  std::unordered_set<Node, NodeHash> open_set_;
  
  NodeComparator getComparator() const
  {
    return NodeComparator(
      const_cast<std::unordered_map<Node, double, NodeHash>*>(&g_values_),
      const_cast<std::unordered_map<Node, double, NodeHash>*>(&rhs_values_),
      const_cast<std::unordered_map<Node, double, NodeHash>*>(&h_values_));
  }
  
  double calculateHeuristic(int x1, int y1, int x2, int y2);
  double calculateKey(const Node& node);
  std::vector<Node> getNeighbors(const Node& node);
  double calculateCost(const Node& from, const Node& to);
  void updateVertex(const Node& node);
  void computeShortestPath();
  bool isValid(int x, int y);
  Node getNode(int x, int y);
};

}  // namespace nav2_dstar_planner

#endif  // NAV2_DSTAR_PLANNER__DSTAR_LITE_HPP_

