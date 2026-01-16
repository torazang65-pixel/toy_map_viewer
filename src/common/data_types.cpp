#include <common/data_types.h>
#include <common/geometry_utils.h>
#include <ros/ros.h>

namespace linemapdraft_builder::data_types {

Point computeAveragePoint(
    const std::vector<Point>& points,
    const std::vector<int>& idxs) {
  float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  float sum_sin = 0.0f, sum_cos = 0.0f;
  uint32_t denominator = 0;
  for (const auto& idx : idxs) {
    const auto& p = points[idx];
    sum_x += p.density * p.x;
    sum_y += p.density * p.y;
    sum_z += p.density * p.z;
    sum_sin += p.density * std::sin(p.yaw);
    sum_cos += p.density * std::cos(p.yaw);
    denominator += p.density;
  }
  float inv = 1.0f / denominator;
  float avg_x = sum_x * inv;
  float avg_y = sum_y * inv;
  float avg_z = sum_z * inv;
  float mean_yaw = std::atan2(sum_sin * inv, sum_cos * inv);
  return {avg_x, avg_y, avg_z, mean_yaw};
}

void build_grid_map(const std::vector<data_types::Point>& points, float grid_size, data_types::GridMap& grid_map) {
  ROS_INFO("Build Grid map...");

  grid_map.clear();

  for (int i = 0; i < points.size(); ++i) {
    const auto& p = points[i];

    int gx = static_cast<int>(std::floor(p.x / grid_size));
    int gy = static_cast<int>(std::floor(p.y / grid_size));
    data_types::GridKey key = {gx, gy};

    grid_map[key].push_back(i);
  }

  ROS_INFO("Grid map built. Total cells: %lu", grid_map.size());
}

// 그리드 주변 이웃 점 인덱스 조회
std::vector<int> getNeighborIndices(
    const std::vector<data_types::Point>& points,
    const data_types::GridMap& grid,
    float grid_size,
    const data_types::Point& center) {
  std::vector<int> neighbor_idxs;
  int gx = static_cast<int>(std::floor(center.x / grid_size));
  int gy = static_cast<int>(std::floor(center.y / grid_size));

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      data_types::GridKey cell{gx + dx, gy + dy};
      auto it = grid.find(cell);
      if (it == grid.end()) continue;
      for (int idx : it->second) {
        if (points[idx].polyline_id != data_types::Dropped &&
            geometry_utils::calculate_dist(points[idx], center) > 0.01) {
          neighbor_idxs.push_back(idx);
        }
      }
    }
  }
  return neighbor_idxs;
}

void build_rtree(point_rtree& rtree, const std::vector<std::vector<data_types::Point>>& polylines) {
  std::vector<point_value> point_vec;
  for (size_t i = 0; i < polylines.size(); ++i) {
    for (size_t j = 0; j < polylines[i].size(); ++j) {
      const auto& point = polylines[i][j];
      point_vec.emplace_back(bg::make<bg_point>(point.x, point.y), RTreeIndex(i, j));
    }
  }
  rtree.insert(point_vec.begin(), point_vec.end());
}

void removeIndex(point_rtree& rtree, size_t id, const std::vector<data_types::Point>& curve) {
  for (size_t i = 0; i < curve.size(); ++i) {
    bg_point p(curve[i].x, curve[i].y);
    rtree.remove(std::make_pair(p, RTreeIndex{id, i}));
  }
}

void insertIndex(point_rtree& rtree, size_t id, const std::vector<data_types::Point>& curve) {
  for (size_t i = 0; i < curve.size(); ++i) {
    bg_point p(curve[i].x, curve[i].y);
    rtree.insert(std::make_pair(p, RTreeIndex{id, i}));
  }
}

}  // namespace linemapdraft_builder::data_types