#include <common/data_types.h>
#include <common/geometry_utils.h>
#include <common/io.h>
#include <linemapdraft_builder/post_processor.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <algorithm>
#include <array>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <fstream>
#include <unordered_map>
#include <vector>

using PointIdx = std::pair<size_t, size_t>;

// 여기서 p1, p2는 original_polylines에서의 인덱스 (id, idx)
struct MergeCandidate {
  PointIdx p1;
  PointIdx p2;
  float cost = 1.0;

  bool operator<(const MergeCandidate& other) const {
    return cost < other.cost;
  }
};

struct PointIdxHash {
  std::size_t operator()(const PointIdx& idx) const {
    return std::hash<size_t>()(idx.first) ^ std::hash<size_t>()(idx.second);
  }
};

namespace {
using namespace linemapdraft_builder;

data_types::Point directionVector(const data_types::Point& from, const data_types::Point& to) {
  data_types::Point d = to - from;
  float len = d.norm();
  if (len > 1e-12) {
    d.x /= len;
    d.y /= len;
    d.z /= len;
  }
  return d;
}

std::optional<float> getMergeCost(const data_types::Point& end1, const data_types::Point& prev1,
                                  const data_types::Point& end2, const data_types::Point& prev2,
                                  float min_dist_th, float max_dist_th,
                                  float min_angle_th, float max_angle_th) {
  float dist = geometry_utils::calculate_dist(end1, end2);
  float dist2 = geometry_utils::calculate_dist(prev1, end2);
  float dist3 = geometry_utils::calculate_dist(end1, prev2);

  if (dist >= dist2 || dist >= dist3) return std::nullopt;
  if (dist >= max_dist_th) return std::nullopt;

  if (dist < min_dist_th) return 0.0f;

  float angle_th;
  angle_th = (max_dist_th - dist) * max_angle_th + (dist - min_dist_th) * min_angle_th;
  angle_th /= (max_dist_th - min_dist_th);

  data_types::Point end_dir = directionVector(end1, end2);
  data_types::Point mean_dir = directionVector(prev1, end1) + directionVector(end2, prev2);
  mean_dir.x *= 0.5;
  mean_dir.y *= 0.5;
  mean_dir.z *= 0.5;

  float cosTheta = mean_dir.dot(end_dir);
  float angleDiff = std::acos(std::max(-1.0f, std::min(1.0f, cosTheta)));

  if (angleDiff < angle_th) {
    return angleDiff / angle_th;
  } else {
    return std::nullopt;
  }
}

std::optional<MergeCandidate> getMergeCosts(const std::vector<std::vector<data_types::Point>>& polylines,
                                            const size_t poly1_id,
                                            const size_t poly2_id,
                                            const float merge_min_dist_th,
                                            const float merge_max_dist_th,
                                            const float merge_min_angle_th,
                                            const float merge_max_angle_th) {
  const std::vector<data_types::Point>& poly1 = polylines[poly1_id];
  const std::vector<data_types::Point>& poly2 = polylines[poly2_id];

  if (poly1_id == poly2_id || poly1.size() < 2 || poly2.size() < 2) {
    return std::nullopt;
  }

  const data_types::Point& p1_start = poly1[0];
  const data_types::Point& p1_start_2 = poly1[1];
  const data_types::Point& p1_end = poly1[poly1.size() - 1];
  const data_types::Point& p1_end_2 = poly1[poly1.size() - 2];
  const data_types::Point& p2_start = poly2[0];
  const data_types::Point& p2_start_2 = poly2[1];
  const data_types::Point& p2_end = poly2[poly2.size() - 1];
  const data_types::Point& p2_end_2 = poly2[poly2.size() - 2];

  PointIdx p1_start_idx = {poly1_id, 0};
  PointIdx p1_end_idx = {poly1_id, poly1.size() - 1};
  PointIdx p2_start_idx = {poly2_id, 0};
  PointIdx p2_end_idx = {poly2_id, poly2.size() - 1};

  struct EndpointPairContext {
    data_types::Point p1;
    data_types::Point p1_adj;
    data_types::Point p2;
    data_types::Point p2_adj;
    PointIdx idx1;
    PointIdx idx2;
  };

  std::array<EndpointPairContext, 4> context_combs = {{
      {p1_end, p1_end_2, p2_start, p2_start_2, p1_end_idx, p2_start_idx},
      {p1_end, p1_end_2, p2_end, p2_end_2, p1_end_idx, p2_end_idx},
      {p1_start, p1_start_2, p2_start, p2_start_2, p1_start_idx, p2_start_idx},
      {p1_start, p1_start_2, p2_end, p2_end_2, p1_start_idx, p2_end_idx},
  }};

  MergeCandidate merge_candidate;
  bool valid = false;
  for (const auto& context : context_combs) {
    std::optional<float> cost = getMergeCost(context.p1, context.p1_adj, context.p2, context.p2_adj,
                                             merge_min_dist_th, merge_max_dist_th,
                                             merge_min_angle_th, merge_max_angle_th);
    if (cost.has_value() && cost.value() < merge_candidate.cost) {
      merge_candidate.p1 = context.idx1;
      merge_candidate.p2 = context.idx2;
      merge_candidate.cost = cost.value();
      valid = true;
    }
  }
  if(valid){
    return merge_candidate;
  }
  else{
    return std::nullopt;
  }
}

std::vector<data_types::Point> getMergedLine(const std::vector<data_types::Point>& line1, const std::vector<data_types::Point>& line2, size_t line1_idx, size_t line2_idx) {
  std::vector<data_types::Point> merged_line;

  std::vector<data_types::Point> line1_copy = line1;
  std::vector<data_types::Point> line2_copy = line2;

  // [...line1] @ [...line2] 을 수행함. 이때 line1의 병합점이 맨 뒤로, line2의 병합점이 맨 앞으로 와야함
  if (line1_idx == 0) {  // 만약 line1의 병합점이 앞에 있다면, 뒤로 보내줌
    std::reverse(line1_copy.begin(), line1_copy.end());
  }  // 만약 line2의 병합점이 뒤에 있다면, 앞으로 보내줌
  if (line2_idx == line2.size() - 1) {
    std::reverse(line2_copy.begin(), line2_copy.end());
  }

  if (geometry_utils::calculate_dist(line1_copy.back(), line2_copy.front()) <= 1.0f){
    line1_copy.pop_back();
  }

  line1_copy.insert(line1_copy.end(), line2_copy.begin(), line2_copy.end());

  return line1_copy;
}
}  // namespace

namespace linemapdraft_builder::post_processor {

std::vector<MergeCandidate> getMergeCandidates(const std::vector<std::vector<data_types::Point>>& polylines,
                                               const float merge_min_dist_th,
                                               const float merge_max_dist_th,
                                               const float merge_min_angle_th,
                                               const float merge_max_angle_th) {
  std::vector<MergeCandidate> merge_candidates;
  for (size_t i = 0; i < polylines.size(); ++i) {
    for (size_t j = i + 1; j < polylines.size(); ++j) {
      std::optional<MergeCandidate> cand = getMergeCosts(polylines, i, j, merge_min_dist_th, merge_max_dist_th,
                                                         merge_min_angle_th, merge_max_angle_th);
      if(cand.has_value()){
        merge_candidates.push_back(std::move(cand.value()));
      }
    }
  }
  std::sort(merge_candidates.begin(), merge_candidates.end());

  return std::move(merge_candidates);
}

std::vector<std::vector<data_types::Point>> mergeAllPolylines(const std::vector<std::vector<data_types::Point>>& original_polylines,
                                                              const float merge_min_dist_th,
                                                              const float merge_max_dist_th,
                                                              const float merge_min_angle_th,
                                                              const float merge_max_angle_th) {
  // initialize merge polylines
  std::unordered_map<size_t, std::vector<data_types::Point>> merged_polylines;
  for (size_t i = 0; i < original_polylines.size(); ++i) {
    if(original_polylines[i].size() >= 2){
      merged_polylines[i] = original_polylines[i];
    }
  }

  std::unordered_map<PointIdx, PointIdx, PointIdxHash> merge_pair;
  std::vector<MergeCandidate> merge_candidates = getMergeCandidates(original_polylines, merge_min_dist_th,
                                                                    merge_max_dist_th, merge_min_angle_th,
                                                                    merge_max_angle_th);

  for (const auto& merge_candidate : merge_candidates) {
    const auto& p1 = merge_candidate.p1;
    const auto& p2 = merge_candidate.p2;
    const auto& cost = merge_candidate.cost;

    if (merge_pair.count(p1) || merge_pair.count(p2)) continue;
    
    merge_pair[p1] = p2;
    merge_pair[p2] = p1;
  }

  while(!merge_pair.empty()){
    auto it = merge_pair.begin();
    const auto p1 = it->first, p2 = it->second;
    merge_pair.erase(p1);
    merge_pair.erase(p2);
    
    if(p1.first == p2.first) continue;

    const auto [line1_id, point1_idx] = p1;
    const auto [line2_id, point2_idx] = p2;

    if(!merged_polylines.count(line1_id) || !merged_polylines.count(line2_id)) continue;

    const auto& line1 = merged_polylines[line1_id];
    const auto& line2 = merged_polylines[line2_id];
    auto line1_size = line1.size(), line2_size = line2.size();

    std::vector<data_types::Point> merged_line = getMergedLine(line1, line2, point1_idx, point2_idx);
    merged_polylines.erase(line2_id);
    merged_polylines[line1_id] = merged_line;
  
    PointIdx merged_back_pt = std::make_pair(line1_id, merged_line.size()-1);
    PointIdx line1_front_pt = std::make_pair(line1_id, 0);
    PointIdx line1_back_pt = std::make_pair(line1_id, line1_size-1);
    PointIdx line2_front_pt = std::make_pair(line2_id, 0);
    PointIdx line2_back_pt = std::make_pair(line2_id, line2_size-1);
    
    // point1의 다른 merge pt를 위한 처리
    if(point1_idx == 0){
      if(merge_pair.count(line1_back_pt)){
        auto target_pt = merge_pair[line1_back_pt];
        merge_pair[target_pt] = line1_front_pt;
        merge_pair[line1_front_pt] = target_pt;
      }
    }

    // point2의 다른 merge pt를 위한 처리
    if(point2_idx == 0){
      if(merge_pair.count(line2_back_pt)){
        auto target_pt = merge_pair[line2_back_pt];
        merge_pair[target_pt] = merged_back_pt;
        merge_pair[merged_back_pt] = target_pt;
      }
    }
    else{
      if(merge_pair.count(line2_front_pt)){
        auto target_pt = merge_pair[line2_front_pt];
        merge_pair[target_pt] = merged_back_pt;
        merge_pair[merged_back_pt] = target_pt;
      }
    } 
  }
  
  // vector형태로 변환 및 리턴
  std::vector<std::vector<data_types::Point>> result;
  result.reserve(merged_polylines.size());

  for (auto& kv : merged_polylines) {
    result.push_back(std::move(kv.second));
  }

  return result;
}

void dropPolylines(
    std::vector<std::vector<data_types::Point>>& polylines,
    const float min_polyline_length) {
  polylines.erase(
      std::remove_if(
          polylines.begin(),
          polylines.end(),
          [min_polyline_length](const std::vector<data_types::Point>& pts) {
            float len = 0.0f;
            for (size_t idx = 1; idx < pts.size(); ++idx) {
              len += geometry_utils::calculate_dist(pts[idx - 1], pts[idx]);
            }
            return len < min_polyline_length;
          }),
      polylines.end());
}

bool build(const PathManager::PathContext& ctx) {
  ros::NodeHandle nh("~");

  // Get parameters from ROS parameter server
  float merge_min_dist_th, merge_max_dist_th, merge_min_angle_th, merge_max_angle_th, min_polyline_length;

  nh.param("merge_min_dist_th", merge_min_dist_th, 1.0f);
  nh.param("merge_max_dist_th", merge_max_dist_th, 10.0f);
  nh.param("merge_min_angle_th", merge_min_angle_th, 0.1f);
  nh.param("merge_max_angle_th", merge_max_angle_th, 0.3f);
  nh.param("min_polyline_length", min_polyline_length, 3.0f);

  ROS_INFO("polyline post processing build start.");

  const std::string polylines_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::SimplePolyline, PathManager::BinaryKind::Polylines);
  const std::string processed_polylines_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::PostProcessed, PathManager::BinaryKind::Polylines);
  const std::string processed_polylines_points_filename = PathManager::getStageBinaryPath(ctx, PathManager::Stage::PostProcessed, PathManager::BinaryKind::Points);

  // Step 1. Load polylines
  std::vector<std::vector<data_types::Point>> polylines;
  if (!io::load_polylines(polylines_filename, polylines)) {
    return false;
  };

  // Step 2. Merge polylines by extrapolate
  ROS_INFO("before merge: %zu", polylines.size());

  std::vector<std::vector<data_types::Point>> merged_polylines =
      mergeAllPolylines(polylines, merge_min_dist_th, merge_max_dist_th, merge_min_angle_th, merge_max_angle_th);
  ROS_INFO("after merge: %zu", merged_polylines.size());

  // Step 3. Drop polylines
  dropPolylines(merged_polylines, min_polyline_length);
  ROS_INFO("after drop: %zu", merged_polylines.size());
  
  // Step 4. Write polylines
  if (!io::write_polylines(processed_polylines_filename, merged_polylines)) {
    return false;
  }
  if (!io::write_points_from_polylines(processed_polylines_points_filename, merged_polylines)) {
    ROS_ERROR("Cannot open processed polyline points output file. (%s)", processed_polylines_points_filename.c_str());
    return false;
  }

  ROS_INFO("done. (%s)", processed_polylines_filename.c_str());
  return true;
}

}  // namespace linemapdraft_builder::post_processor
