//#include "astar_planner.hpp"
#include "planner/global/astar_planner.hpp"
#include <cmath>
#include <functional>
#include <iostream>

// GridPoint operator==
bool GridPoint::operator==(const GridPoint &other) const {
  return x == other.x && y == other.y;
}

// std::hash<GridPoint> 특수화 구현
namespace std {
std::size_t hash<GridPoint>::operator()(const GridPoint &p) const {
  return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
}
}

// NodeRecord operator>
bool NodeRecord::operator>(const NodeRecord& other) const {
  return f_score > other.f_score;
}

void AstarPlanner::setMap(const std::vector<int8_t>& data, int width, int height, double resolution) {
  map_data_ = data;
  map_width_ = width;
  map_height_ = height;
  resolution_ = resolution;

  std::cout << "[AstarPlanner] Map received: width=" << map_width_
            << ", height=" << map_height_
            << ", resolution=" << resolution_ << std::endl;

  std::cout << "[AstarPlanner] Margin cells: " << margin_cells_ << std::endl;

  updateMarginCells();
}

void AstarPlanner::setMinSafeDistance(double min_safe_distance) {
  min_safe_distance_ = min_safe_distance;
  updateMarginCells();
}

void AstarPlanner::setAllowUnknownArea(bool allow_unknown) {
  allow_unknown_area_ = allow_unknown;
}

void AstarPlanner::updateMarginCells() {
  if (resolution_ > 0) {
    margin_cells_ = std::max(1, static_cast<int>(min_safe_distance_ / resolution_));
    std::cout << "[AstarPlanner] updateMarginCells: margin_cells_=" << margin_cells_ << std::endl;
  } else {
    margin_cells_ = 1;
  }
}

bool AstarPlanner::isValid(const GridPoint& p) const {
  return (p.x >= 0 && p.x < map_width_ && p.y >= 0 && p.y < map_height_);
}

bool AstarPlanner::isSafe(const GridPoint& p) const {
  if (map_data_.empty()) return false;

  for (int dy = -margin_cells_; dy <= margin_cells_; ++dy) {
    int ny = p.y + dy;
    if (ny < 0 || ny >= map_height_) continue;

    for (int dx = -margin_cells_; dx <= margin_cells_; ++dx) {
      int nx = p.x + dx;
      if (nx < 0 || nx >= map_width_) continue;

      int idx = ny * map_width_ + nx;
      int val = map_data_[idx];

      if (val == -1 && !allow_unknown_area_) return false;
      if (val >= 50) return false;
    }
  }
  return true;
}

double AstarPlanner::heuristic(const GridPoint& a, const GridPoint& b) const {
  return std::hypot(b.x - a.x, b.y - a.y);
}

std::vector<GridPoint> AstarPlanner::planPath(const GridPoint& start, const GridPoint& goal) {
  std::vector<GridPoint> path;

  if (map_data_.empty()) {
    std::cout << "[AstarPlanner] map_data_ is empty, returning empty path" << std::endl;
    return path;
  }

  if (!isValid(start) || !isValid(goal)) {
    std::cout << "[AstarPlanner] start or goal is invalid, returning empty path" << std::endl;
    return path;
  }

  std::priority_queue<NodeRecord, std::vector<NodeRecord>, std::greater<NodeRecord>> open_set;
  std::unordered_map<GridPoint, GridPoint> came_from;
  std::unordered_map<GridPoint, double> g_score;

  open_set.push({start, heuristic(start, goal), 0.0});
  g_score[start] = 0.0;

  std::vector<GridPoint> directions = {
    {1,0}, {-1,0}, {0,1}, {0,-1},
    {1,1}, {1,-1}, {-1,1}, {-1,-1}
  };

  while (!open_set.empty()) {
    NodeRecord current = open_set.top();
    open_set.pop();

    #if __LOG__
    std::cout << "[AstarPlanner] Checking node (" << current.point.x << ", " << current.point.y << ") with f_score=" << current.f_score << std::endl;
    #endif

    if (current.point == goal) {
      std::cout << "[AstarPlanner] Goal reached, reconstructing path" << std::endl;
      GridPoint step = current.point;
      while (!(step == start)) {
        path.push_back(step);
        step = came_from[step];
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const auto& dir : directions) {
      GridPoint neighbor{current.point.x + dir.x, current.point.y + dir.y};

      if (!isValid(neighbor)) {
        std::cout << "[AstarPlanner] Neighbor (" << neighbor.x << ", " << neighbor.y << ") invalid, skipping" << std::endl;
        continue;
      }
      // if (!isSafe(neighbor)) continue; // 필요하면 활성화

      // 대각선 이동 시 꼭짓점 끼임 방지
      if (dir.x != 0 && dir.y != 0) {
        GridPoint neighbor_x{current.point.x + dir.x, current.point.y};
        GridPoint neighbor_y{current.point.x, current.point.y + dir.y};
        if (!isSafe(neighbor_x) || !isSafe(neighbor_y)) {
          #if __LOG__
          std::cout << "[AstarPlanner] Diagonal neighbor blocked at (" 
                    << neighbor_x.x << ", " << neighbor_x.y << ") or (" 
                    << neighbor_y.x << ", " << neighbor_y.y << "), skipping" << std::endl;
          #endif
          continue;
        }
      }

      double tentative_g = current.g_score + ((dir.x == 0 || dir.y == 0) ? 1.0 : 1.414);

      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        came_from[neighbor] = current.point;
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(neighbor, goal);
        open_set.push({neighbor, f, tentative_g});
        #if __LOG__
        std::cout << "[AstarPlanner] Added neighbor (" << neighbor.x << ", " << neighbor.y << ") to open set with f=" << f << std::endl;
        #endif
      }
    }
  }

  std::cout << "[AstarPlanner] No path found, returning empty path" << std::endl;
  return path;
}

