#include <common/data_types.h>
#include <common/geometry_utils.h>
#include <common/io.h>
#include <linemapdraft_builder/polyline_generator.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <numeric>
#include <queue>
#include <utility>
#include <vector>

namespace {
using namespace linemapdraft_builder;

// Compute the IQR bounds for yaw values
std::pair<float, float> computeYawIQRBounds(std::vector<float> yaw_vals) {
  std::sort(yaw_vals.begin(), yaw_vals.end());
  size_t m = yaw_vals.size();
  if (m == 0) return {0.0, 0.0};  // should not happen
  float Q1 = yaw_vals[m / 4];
  float Q3 = yaw_vals[(3 * m) / 4];
  float IQR = Q3 - Q1;
  float lower = Q1 - 1.5f * IQR;
  float upper = Q3 + 1.5f * IQR;
  return std::make_pair(lower, upper);
}

// Yaw outlier filtering
std::vector<int> filterYawIQR(std::vector<data_types::Point>& points,
                              const std::vector<int>& cand_idxs) {
  if (cand_idxs.empty()) return {};

  std::vector<float> yaw_vals;
  yaw_vals.reserve(cand_idxs.size());
  std::transform(cand_idxs.begin(), cand_idxs.end(), std::back_inserter(yaw_vals),
                 [&points](int idx) { return points[idx].yaw; });

  const auto [lower, upper] = computeYawIQRBounds(yaw_vals);

  std::vector<int> filtered_idxs;
  for (const auto& idx : cand_idxs) {
    auto& cand = points[idx];
    if (cand.yaw >= lower && cand.yaw <= upper) {
      filtered_idxs.push_back(idx);
    } else {
      cand.polyline_id = data_types::Dropped;
    }
  }

  return filtered_idxs;
}

// 충분히 큰 Sequence에 대해 Sequence의 이전 점을 반영한 yaw값
float getSmoothedYaw(const std::vector<data_types::Point>& sequence) {
  auto center = sequence.back();
  float smoothed_center_yaw = center.yaw;
  if (sequence.size() >= 4) {
    auto prev = sequence[sequence.size() - 2];
    float yaw_diff = center.yaw - prev.yaw;
    float diff_rate = std::min(10.0f, (float)(sequence.size()) - 1);
    smoothed_center_yaw = center.yaw + yaw_diff * 0.05 * diff_rate;
  }
  return smoothed_center_yaw;
}

// 선분 (center, next_Point)에 drop_width 만큼의 원기둥 영역을 sweep, drop
void sweepAndDrop(std::vector<data_types::Point>& points,
                  std::vector<int> neighbor_idxs,
                  const data_types::Point& center,
                  const data_types::Point& next_point,
                  float drop_width) {
  for (int idx : neighbor_idxs) {
    auto& nbr = points[idx];
    bool inside = false;
    float pd = geometry_utils::perpendicular_distance(center, next_point, nbr, inside);
    if (inside && pd < drop_width) {
      nbr.polyline_id = data_types::Dropped;
    }
  }
}

// neighbor_idxs 중 filter 조건을 만족하는 점을 리턴
std::vector<int> getCandidates(const std::vector<int>& neighbor_idxs,
                               const std::function<bool(int)>& filter) {
  std::vector<int> out;
  std::copy_if(neighbor_idxs.begin(), neighbor_idxs.end(),
               std::back_inserter(out), filter);
  return out;
}

// if any of previous point can be connected by cone/cylinder, then it's polygon!
bool checkPolygon(std::vector<data_types::Point>& sequence,
                  const std::function<bool(int)>& polygonCylFilter) {
  // check for all pts in sequence except the last one.
  std::vector<int> idxs(sequence.size() - 1);
  std::iota(idxs.begin(), idxs.end(), 0);

  std::vector<int> cand_idxs = getCandidates(idxs, polygonCylFilter);
  if (cand_idxs.size() > 0) {
    sequence.push_back(sequence[cand_idxs.front()]);
  }

  return cand_idxs.size() > 0;
}

void updateEWMA(float& ewma_yaw_x, float& ewma_yaw_y, float& ewma_vz,
                float alpha, float beta,
                const data_types::Point& center,
                const data_types::Point& avg) {
  float heading = geometry_utils::calculate_heading(center, avg);
  ewma_yaw_x = (1 - alpha) * ewma_yaw_x + alpha * (beta * std::cos(heading) + (1 - beta) * std::cos(avg.yaw));
  ewma_yaw_y = (1 - alpha) * ewma_yaw_y + alpha * (beta * std::sin(heading) + (1 - beta) * std::sin(avg.yaw));
  float dxy = std::hypot(avg.x - center.x, avg.y - center.y);
  ewma_vz = (1 - alpha) * ewma_vz + alpha * ((avg.z - center.z) / dxy);
}

}  // namespace

namespace linemapdraft_builder::polyline_generator {

bool checkAndClosePolygon(std::vector<data_types::Point>& points,
                          std::vector<data_types::Point>& sequence,
                          std::vector<int> neighbor_idxs,
                          const data_types::Point& center,
                          const float smoothed_center_yaw,
                          const float polygon_dist_th,
                          const float search_width,
                          const float drop_width) {
  data_types::Point polygon_hp = center + polygon_dist_th *
                                              data_types::Point{std::cos(smoothed_center_yaw), std::sin(smoothed_center_yaw), center.vz};

  auto polygon_CylFilter = [&](int nbr_idx) {
    const auto& nbr = sequence[nbr_idx];
    float distance = geometry_utils::calculate_dist(center, nbr);
    if (distance >= polygon_dist_th) return false;

    bool inside = false;
    float pd = geometry_utils::perpendicular_distance(center, polygon_hp, nbr, inside);
    return inside && pd < search_width;
  };

  bool polygon = checkPolygon(sequence, polygon_CylFilter);
  if (polygon) {
    sweepAndDrop(points, neighbor_idxs, center, sequence.front(), drop_width);
  }
  return polygon;
}

std::optional<data_types::Point> getNextPoint(std::vector<data_types::Point>& points,
                                              const std::vector<int>& neighbor_idxs,
                                              const data_types::Point& center,
                                              float smoothed_center_yaw,
                                              float neighbor_dist_thresh,
                                              float search_width,
                                              uint32_t min_density) {
  data_types::Point hp = center + neighbor_dist_thresh *
                                      data_types::Point{std::cos(smoothed_center_yaw),
                                                        std::sin(smoothed_center_yaw),
                                                        center.vz};

  auto cylFilter = [&](int nbr_idx) {
    const auto& nbr = points[nbr_idx];
    float distance = geometry_utils::calculate_dist(center, nbr);
    if (distance >= neighbor_dist_thresh) return false;

    bool inside = false;
    float pd = geometry_utils::perpendicular_distance(center, hp, nbr, inside);
    return inside && pd < search_width;
  };

  std::vector<int> cand_idxs = getCandidates(neighbor_idxs, cylFilter);

  uint32_t num_cand_points = 0;
  for(const auto& idx: cand_idxs){
    const auto& pt = points[idx];
    num_cand_points += pt.density;
  }
  if (num_cand_points < min_density) return std::nullopt;

  auto yaw_filtered_idxs = filterYawIQR(points, cand_idxs);
  if (yaw_filtered_idxs.empty()) return std::nullopt;

  data_types::Point next_pt = data_types::computeAveragePoint(points, yaw_filtered_idxs);
  return next_pt;
}

void extendPolyline(std::vector<data_types::Point>& points,
                    const data_types::GridMap& grid,
                    const float grid_size,
                    const float neighbor_dist_thresh,
                    const float cylinder_search_width,
                    const float alpha,
                    const float beta,
                    const float drop_width,
                    const uint32_t min_density,
                    int current_polyline_idx,
                    std::vector<data_types::Point>& sequence,
                    bool reverse,
                    bool& is_polygon) {
  float init_yaw = sequence.front().yaw;
  float ewma_x = std::cos(init_yaw);
  float ewma_y = std::sin(init_yaw);
  float ewma_vz = 0.0f;

  while (true) {
    data_types::Point center = sequence.back();
    std::vector<int> neighbor_idxs = data_types::getNeighborIndices(points, grid, grid_size, center);

    float smoothed_center_yaw = getSmoothedYaw(sequence);
    // 이전 yaw값을 반영한 yaw값과 현재 점의 yaw값의 차이가 클수록 곡선구간이라는 뜻, search_width를 키운다.
    float search_width = cylinder_search_width * std::clamp(std::abs(smoothed_center_yaw - center.yaw) * 5, 0.5f, 1.0f);

    // 폴리곤인지를 먼저 체크한다.
    float polygon_dist_th = 2.0f;
    is_polygon = checkAndClosePolygon(points, sequence, neighbor_idxs, center, smoothed_center_yaw, polygon_dist_th, search_width, drop_width);
    if (is_polygon) {
      break;
    }

    // 다음 점을 탐색 & 평균낸 Next Point를 찾는다.
    std::optional<data_types::Point> next_point_opt = getNextPoint(points, neighbor_idxs, center, smoothed_center_yaw, neighbor_dist_thresh, search_width, min_density);
    if (!next_point_opt) {
      break;  // No more points to extend
    }

    data_types::Point next_pt = *next_point_opt;

    if (reverse) {
      // 역방향 진행을 위해 yaw 반전
      next_pt.yaw += M_PI;
      next_pt.vz *= -1;
    }

    updateEWMA(ewma_x, ewma_y, ewma_vz, alpha, beta, center, next_pt);
    next_pt.yaw = std::atan2(ewma_y, ewma_x);
    next_pt.vz = ewma_vz;
    next_pt.polyline_id = current_polyline_idx;

    sequence.push_back(next_pt);
    sweepAndDrop(points, neighbor_idxs, center, next_pt, drop_width);
  }
}

void generate_polyline(std::vector<data_types::Point>& points,
                       std::vector<std::vector<data_types::Point>>& polylines,
                       const data_types::GridMap& grid,
                       const float grid_size,
                       const uint32_t min_density,
                       const float neighbor_dist_thresh,
                       const float cylinder_search_width,
                       const float alpha,
                       const float beta,
                       const float drop_width) {
  int32_t current_polyline_idx = 0;
  for (auto& root : points) {
    // Step 0. check whether the point is already classified
    if (root.density < min_density || root.polyline_id != data_types::Unclassified) continue;

    // Step 1. bidirectional polyline extension
    data_types::Point opp_root = root;
    opp_root.yaw += M_PI;
    opp_root.vz *= -1;
    root.polyline_id = current_polyline_idx;
    opp_root.polyline_id = current_polyline_idx;
    std::vector<data_types::Point> direct{root}, opposite{opp_root};

    bool polygon = false;
    extendPolyline(points, grid, grid_size,
                   neighbor_dist_thresh, cylinder_search_width,
                   alpha, beta, drop_width, min_density,
                   current_polyline_idx,
                   direct, false, polygon);
    if (!polygon) {
      extendPolyline(points, grid, grid_size,
                     neighbor_dist_thresh, cylinder_search_width,
                     alpha, beta, drop_width, min_density,
                     current_polyline_idx,
                     opposite, true, polygon);
    }

    // Step 2. Collect points from 'direct' and 'opposite' to make polyline
    std::vector<data_types::Point> polyline;
    for (auto it = direct.end() - 1; it > direct.begin(); it--) {
      polyline.push_back(*it);
    }
    for (auto p : opposite) {
      p.yaw += M_PI;
      p.vz = -p.vz;
      polyline.push_back(p);
    }
    polylines.push_back(polyline);
    current_polyline_idx++;
  }

  ROS_DEBUG("Total polylines assigned: %d", current_polyline_idx);
}

bool build(const PathManager::PathContext& ctx) {
  ros::NodeHandle nh("~");

  float neighbor_dist_thresh, cylinder_search_width, alpha, beta, drop_width;

  nh.param("neighbor_dist_thresh", neighbor_dist_thresh, 2.0f);
  nh.param("cylinder_search_width", cylinder_search_width, 1.0f);
  nh.param("alpha", alpha, 0.7f);
  nh.param("beta", beta, 0.9f);
  nh.param("drop_width", drop_width, 1.0f);

  ROS_INFO("polyline build start.");

  const std::string line_voxel_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::LineVoxel, PathManager::BinaryKind::Points);
  const std::string polyline_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::Polyline, PathManager::BinaryKind::Polylines);
  const std::string polyline_points_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::Polyline, PathManager::BinaryKind::Points);

  // Step 1. load points
  std::vector<data_types::Point> points;
  if (!io::load_points(line_voxel_filename, points)) {
    return false;
  };
  std::sort(points.begin(), points.end(), [](const data_types::Point& a, const data_types::Point& b) {
    return a.density > b.density;
  });

  uint32_t min_density = 0;
  if(!points.empty()) {
    min_density = points[points.size()/2].density;
  }

  // Step 2. build gridmap
  data_types::GridMap grid;
  float grid_size = neighbor_dist_thresh * 0.9f;
  build_grid_map(points, grid_size, grid);

  // Step 3. generate polylines
  std::vector<std::vector<data_types::Point>> polylines;
  generate_polyline(points, polylines, grid, grid_size, min_density, neighbor_dist_thresh, cylinder_search_width, alpha, beta, drop_width);

  // Step 4. write polylines
  if (!io::write_polylines(polyline_filename, polylines)) {
    return false;
  }

  // Step 5. write points from polylines
  if (!io::write_points_from_polylines(polyline_points_filename, polylines)) {
    return false;
  }

  ROS_INFO("done. (%s)", polyline_filename.c_str());
  return true;
}

}  // namespace linemapdraft_builder::polyline_generator
