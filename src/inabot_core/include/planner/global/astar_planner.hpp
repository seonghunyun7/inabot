#ifndef ASTAR_PLANNER_HPP
#define ASTAR_PLANNER_HPP

#include <vector>
#include <unordered_map>
#include <queue>
#include <cmath>

struct GridPoint {
  int x, y;
  bool operator==(const GridPoint &other) const;
};

namespace std {
template <>
struct hash<GridPoint> {
  std::size_t operator()(const GridPoint &p) const;
};
}

struct NodeRecord {
  GridPoint point;
  double f_score;
  double g_score;

  bool operator>(const NodeRecord& other) const;
};

class AstarPlanner
{
public:
  AstarPlanner() = default;

  void setMap(const std::vector<int8_t>& data, int width, int height, double resolution);
  void setMinSafeDistance(double min_safe_distance);
  void setAllowUnknownArea(bool allow_unknown);

  std::vector<GridPoint> planPath(const GridPoint& start, const GridPoint& goal);

private:
  bool isValid(const GridPoint& p) const;
  bool isSafe(const GridPoint& p) const;
  double heuristic(const GridPoint& a, const GridPoint& b) const;

  std::vector<int8_t> map_data_;
  int map_width_ = 0;
  int map_height_ = 0;
  double resolution_ = 0.0;

  double min_safe_distance_ = 0.3;
  bool allow_unknown_area_ = false;

  int margin_cells_ = 1;

  void updateMarginCells();
};

#endif // ASTAR_PLANNER_HPP
