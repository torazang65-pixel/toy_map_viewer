#include <common/data_types.h>
#include <common/geometry_utils.h>
#include <common/io.h>
#include <realtime_line_generator/polyline_simplifier.h> 
#include <ros/package.h>
#include <ros/ros.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

namespace linemapdraft_builder::polyline_simplifier {

// Recursive RDP
void rdpRec(const std::vector<data_types::Point>& pts, int i0, int i1, float eps, std::vector<bool>& keep) {
  if (i1 <= i0 + 1) return;
  // find max distance
  float maxDist = 0.0f;
  int idx = i0;
  for (int i = i0 + 1; i < i1; i++) {
    float d = geometry_utils::perpendicular_distance(pts[i0], pts[i1], pts[i]);
    if (d > maxDist) {
      maxDist = d;
      idx = i;
    }
  }
  if (maxDist > eps) {
    keep[idx] = true;
    rdpRec(pts, i0, idx, eps, keep);
    rdpRec(pts, idx, i1, eps, keep);
  }
}

// RDP wrapper
void rdp(std::vector<data_types::Point>& polyline, float eps) {
  std::vector<data_types::Point> pts = polyline;
  int n = static_cast<int>(pts.size());
  if (n <= 2) return;

  // Determine which points to keep
  std::vector<bool> keep(n, false);
  keep[0] = keep[n - 1] = true;
  rdpRec(pts, 0, n - 1, eps, keep);

  // Rebuild polyline
  polyline.clear();
  for (int i = 0; i < n; i++) {
    if (keep[i]) polyline.push_back(pts[i]);
  }
  return;
}

struct Node {
  int idx;
  int prev;
  int next;
  float area;
  bool alive;
};

void vw(std::vector<data_types::Point>& polyline,
        float area_thresh) {
  std::vector<data_types::Point> pts = polyline;
  int n = static_cast<int>(pts.size());
  if (n <= 2) return;

  // 0) clear output container
  polyline.clear();

  // 1) Initialize Point Nodes
  std::vector<Node> nodes;
  nodes.reserve(n);
  for (int i = 0; i < n; ++i) {
    nodes.push_back(Node{
        /*idx*/ i,
        /*prev*/ (i > 0 ? i - 1 : -1),
        /*next*/ (i < n - 1 ? i + 1 : -1),
        /*area*/ std::numeric_limits<float>::infinity(),
        /*alive*/ true});
  }

  // 2) Triangle area computation
  auto computeArea = [&](int i) {
    int p = nodes[i].prev;
    int q = nodes[i].next;
    if (p < 0 || q < 0) return std::numeric_limits<float>::infinity();
    const auto& A = pts[p];
    const auto& B = pts[i];
    const auto& C = pts[q];
    // area = 0.5 * |(A - B) × (C - B)|
    return 0.5f * ((A - B).cross(C - B)).norm();
  };

  // 3) Initial area computation and ordered set construction
  std::set<std::pair<float, int>> areaSet;
  for (int i = 1; i < n - 1; ++i) {
    nodes[i].area = computeArea(i);
    areaSet.emplace(nodes[i].area, i);
  }

  // 4) Remove points below threshold
  while (!areaSet.empty() && areaSet.begin()->first < area_thresh) {
    auto [a, i] = *areaSet.begin();
    areaSet.erase(areaSet.begin());
    nodes[i].alive = false;

    int p = nodes[i].prev;
    int q = nodes[i].next;
    // Erase p, q old areas
    if (p > 0 && p < n - 1) areaSet.erase({nodes[p].area, p});
    if (q > 0 && q < n - 1) areaSet.erase({nodes[q].area, q});
    // Re-link
    nodes[p].next = q;
    nodes[q].prev = p;
    // Compute and insert new areas
    nodes[p].area = computeArea(p);
    nodes[q].area = computeArea(q);
    if (p > 0 && p < n - 1)
      areaSet.emplace(nodes[p].area, p);
    if (q > 0 && q < n - 1)
      areaSet.emplace(nodes[q].area, q);
  }

  // 5) 남은 점 순차 복사
  for (const auto& node : nodes) {
    if (node.alive) polyline.push_back(pts[node.idx]);
  }
}

bool build(const PathManager::PathContext& ctx,
           const PathManager::Stage prev_stage,
           const PathManager::Stage cur_stage,
           const bool vw_flag,
           const bool rdp_flag) {
  ros::NodeHandle nh("~");

  // Get parameters from ROS parameter server
  float VW_eps, RDP_eps;

  if (vw_flag && rdp_flag) {
    // Second simplification (both VW and RDP)
    nh.param("VW_eps_2", VW_eps, 3.0f);
    nh.param("RDP_eps", RDP_eps, 0.5f);
  } else if (vw_flag) {
    // First simplification (VW only)
    nh.param("VW_eps_1", VW_eps, 0.5f);
    RDP_eps = 0.0f;  // Not used
  } else {
    // Default values
    VW_eps = 0.5f;
    RDP_eps = 0.5f;
  }

  // Step 1. Read input polyline file
  // Step 2. Generate polyline
  // Step 3. Write output polyline file

  ROS_INFO("polyline simplification start.");

  const std::string polyline_filename = PathManager::getStageBinaryPath(ctx, prev_stage, PathManager::BinaryKind::Polylines);
  const std::string simple_polyline_filename = PathManager::getStageBinaryPath(ctx, cur_stage, PathManager::BinaryKind::Polylines);
  const std::string simple_polyline_points_filename = PathManager::getStageBinaryPath(ctx, cur_stage, PathManager::BinaryKind::Points);

  // Step 1. Read input polyline file
  std::vector<std::vector<data_types::Point>> input_polylines;
  if (!io::load_polylines(polyline_filename, input_polylines)) {
    ROS_ERROR("Cannot open polyline input file. (%s)", polyline_filename.c_str());
    return false;
  }

  // Step 2. Generate polyline
  std::vector<std::vector<data_types::Point>> simple_polylines;
  for (const auto& input_polyline : input_polylines) {
    std::vector<data_types::Point> simple_polyline = input_polyline;
    if (vw_flag) vw(simple_polyline, VW_eps);
    if (rdp_flag) rdp(simple_polyline, RDP_eps);
    simple_polylines.push_back(simple_polyline);
  }

  // Step 3. Write output polyline file
  if (!io::write_polylines(simple_polyline_filename, simple_polylines)) {
    ROS_ERROR("Cannot open polyline output file. (%s)", simple_polyline_filename.c_str());
    return false;
  }

  if (!io::write_points_from_polylines(simple_polyline_points_filename, simple_polylines)) {
    ROS_ERROR("Cannot open polyline points output file. (%s)", simple_polyline_points_filename.c_str());
    return false;
  }

  ROS_INFO("done. (%s) wrote %zu polylines", simple_polyline_filename.c_str(), simple_polylines.size());
  return true;
}
}  // namespace linemapdraft_builder::polyline_simplifier
