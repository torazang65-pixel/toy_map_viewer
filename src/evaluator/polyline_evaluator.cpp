#include <common/data_types.h>
#include <common/geometry_utils.h>
#include <common/io.h>
#include <evaluator/polyline_evaluator.h>
#include <ros/package.h>
#include <ros/ros.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <vector>

namespace linemapdraft_builder::polyline_evaluator {

float chamferDistance(
    const std::vector<data_types::Point> &preds,
    const std::vector<data_types::Point> &gts,
    float chamfer_dist_th) {
  float sumPreds = 0.0;
  for (const auto &p : preds) {
    float minDist = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < gts.size(); ++i) {
      bool inside = false;
      minDist = std::min(minDist, geometry_utils::perpendicular_distance_bev(gts[i], gts[i + 1], p, inside));
    }
    sumPreds += minDist;
  }

  float sumGTs = 0.0;
  for (const auto &p : gts) {
    float minDist = std::numeric_limits<float>::max();
    for (size_t i = 0; i + 1 < preds.size(); ++i) {
      bool inside = false;
      minDist = std::min(minDist, geometry_utils::perpendicular_distance_bev(preds[i], preds[i + 1], p, inside));
    }
    sumGTs += minDist;
  }

  // Chamfer dist1: pred->gt, dist2: gt->pred
  float dist1 = sumPreds / preds.size(), dist2 = sumGTs / gts.size();

  // Standard Chamfer: average the two sums
  float standard_dist = (dist1 + dist2) * 0.5;

  return (dist1 < chamfer_dist_th) && (dist2 < chamfer_dist_th) ? standard_dist : std::numeric_limits<float>::max();
}

void hungarian(const std::vector<std::vector<float>> &costMatrix,
               std::vector<int> &assignment) {
  // gpt gen code. Not verified

  int n = costMatrix.size();
  int m = costMatrix[0].size();
  int max_dim = std::max(n, m);
  std::vector<std::vector<float>> cost(max_dim, std::vector<float>(max_dim, 0));

  // Fill square cost matrix
  for (int i = 0; i < max_dim; ++i)
    for (int j = 0; j < max_dim; ++j)
      cost[i][j] = (i < n && j < m) ? costMatrix[i][j] : 0;

  std::vector<float> u(max_dim + 1), v(max_dim + 1);
  std::vector<int> p(max_dim + 1), way(max_dim + 1);

  for (int i = 1; i <= max_dim; ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<float> minv(max_dim + 1, std::numeric_limits<float>::infinity());
    std::vector<char> used(max_dim + 1, false);
    do {
      used[j0] = true;
      int i0 = p[j0], j1 = 0;
      float delta = std::numeric_limits<float>::infinity();
      for (int j = 1; j <= max_dim; ++j) {
        if (!used[j]) {
          float cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }
      for (int j = 0; j <= max_dim; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  assignment.assign(n, -1);
  for (int j = 1; j <= max_dim; ++j)
    if (p[j] <= n && j <= m)
      assignment[p[j] - 1] = j - 1;
}

void buildCostMatrix(
    const std::vector<std::vector<data_types::Point>> &preds,
    const std::vector<std::vector<data_types::Point>> &gts,
    std::vector<std::vector<float>> &cost,
    float chamfer_dist_th) {
  cost.resize(preds.size(), std::vector<float>(gts.size()));
  for (size_t i = 0; i < preds.size(); ++i) {
    for (size_t j = 0; j < gts.size(); ++j) {
      cost[i][j] = chamferDistance(preds[i], gts[j], chamfer_dist_th);
    }
  }
}

void evaluate_AP(const std::vector<std::vector<data_types::Point>> &preds,
                 const std::vector<std::vector<data_types::Point>> &gts,
                 float chamfer_dist_th) {
  // Line wise matching
  std::vector<std::vector<float>> cost;
  buildCostMatrix(preds, gts, cost, chamfer_dist_th);

  std::vector<int> assignment;
  hungarian(cost, assignment);

  // Count matches
  int tp = 0;
  int fp = 0;
  int fn = 0;
  std::vector<bool> gtMatched(gts.size(), false);
  for (size_t i = 0; i < preds.size(); ++i) {
    int j = assignment[i];
    if (j >= 0 && j < (int)gts.size()) {
      if (cost[i][j] <= chamfer_dist_th) {
        tp++;
        gtMatched[j] = true;
      } else {
        fp++;
      }
    } else {
      fp++;
    }
  }
  for (bool matched : gtMatched)
    if (!matched)
      fn++;

  float precision = tp / float(tp + fp);
  float recall = tp / float(tp + fn);
  float f1 = (precision + recall) > 0 ? 2.0 * precision * recall / (precision + recall) : 0.0;

  std::cout << "\n===== AP Evaluation Results =====\n";
  std::cout << std::setw(12) << "Metric" << " | " << std::setw(8) << "Value" << "\n";
  std::cout << "------------------------------\n";
  std::cout << std::setw(12) << "TP" << " | " << std::setw(8) << tp << "\n";
  std::cout << std::setw(12) << "FP" << " | " << std::setw(8) << fp << "\n";
  std::cout << std::setw(12) << "FN" << " | " << std::setw(8) << fn << "\n";
  std::cout << std::setw(12) << "Precision" << " | " << std::setw(8) << std::fixed << std::setprecision(3) << precision << "\n";
  std::cout << std::setw(12) << "Recall" << " | " << std::setw(8) << recall << "\n";
  std::cout << std::setw(12) << "F1-score" << " | " << std::setw(8) << f1 << "\n";
  std::cout << "------------------------------\n";
  std::cout << precision * 100 << "% of draft lines are matched!!" << std::endl;
  std::cout << recall * 100 << "% of gt lines are matched!!" << std::endl;
  std::cout << "==============================\n\n";
}

void evaluate_iou(const std::vector<std::vector<data_types::Point>> &preds,
                  const std::vector<std::vector<data_types::Point>> &gts,
                  float point_iou_th) {
  // Point wise matching
  std::vector<data_types::Point> flat_preds, flat_gts;
  for (const auto &line : preds) {
    flat_preds.insert(flat_preds.end(), line.begin(), line.end());
  }
  for (const auto &line : gts) {
    flat_gts.insert(flat_gts.end(), line.begin(), line.end());
  }

  std::vector<bool> gt_matched(flat_gts.size(), false);
  std::vector<bool> pred_matched(flat_preds.size(), false);

  // greedy matching
  for (size_t i = 0; i < flat_preds.size(); ++i) {
    double best_dist = point_iou_th;
    int best_gt_idx = -1;

    for (size_t j = 0; j + 1 < flat_gts.size(); ++j) {
      if (gt_matched[j])
        continue;  // already matched
      bool inside = false;
      double dist = geometry_utils::perpendicular_distance_bev(flat_gts[j], flat_gts[j + 1], flat_preds[i], inside);
      if (inside && dist <= best_dist) {
        best_dist = dist;
        best_gt_idx = j;
      }
    }

    if (best_gt_idx != -1) {
      pred_matched[i] = true;
      gt_matched[best_gt_idx] = true;
    }
  }

  int pred_cnt = flat_preds.size(), gt_cnt = flat_gts.size();
  int TP = 0;
  for (const auto &matched : pred_matched) {
    if (matched)
      TP++;
  }
  float precision = (pred_cnt > 0) ? ((float)TP / pred_cnt) : 0.0;
  float recall = (gt_cnt > 0) ? ((float)TP / gt_cnt) : 0.0;
  float f1 = (precision + recall) > 0 ? 2.0 * precision * recall / (precision + recall) : 0.0;

  std::cout << "\n===== IOU Evaluation Results =====\n";
  std::cout << std::setw(12) << "Metric" << " | " << std::setw(8) << "Value" << "\n";
  std::cout << "------------------------------\n";
  std::cout << std::setw(12) << "TP" << " | " << std::setw(8) << TP << "\n";
  std::cout << std::setw(12) << "FP" << " | " << std::setw(8) << pred_cnt - TP << "\n";
  std::cout << std::setw(12) << "FN" << " | " << std::setw(8) << gt_cnt - TP << "\n";
  std::cout << std::setw(12) << "Precision" << " | " << std::setw(8) << std::fixed << std::setprecision(3) << precision << "\n";
  std::cout << std::setw(12) << "Recall" << " | " << std::setw(8) << recall << "\n";
  std::cout << std::setw(12) << "F1-score" << " | " << std::setw(8) << f1 << "\n";
  std::cout << "------------------------------\n";
  std::cout << precision * 100 << "% of draft lines are valid!!" << std::endl;
  std::cout << recall * 100 << "% of gt lines are generated!!" << std::endl;
  std::cout << "==============================\n\n";
}

data_types::Point interpolate_linear(const data_types::Point &p1, const data_types::Point &p2, float t) {
  data_types::Point out;
  out.x = p1.x + t * (p2.x - p1.x);
  out.y = p1.y + t * (p2.y - p1.y);
  out.z = p1.z + t * (p2.z - p1.z);
  out.yaw = std::atan2(p2.y - p1.y, p2.x - p1.x);
  return out;
}

std::vector<data_types::Point> resample_single_polyline(const std::vector<data_types::Point> &polyline, float sampling_distance) {
  std::vector<data_types::Point> result;
  if (polyline.size() < 2)
    return polyline;

  for (size_t i = 0; i + 1 < polyline.size(); ++i) {
    const auto &p1 = polyline[i];
    const auto &p2 = polyline[i + 1];

    result.push_back(p1);

    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    // todo: 여기도 z축은 빼둠
    float dist = std::sqrt(dx * dx + dy * dy);

    int num_insert = static_cast<int>(std::floor(dist / sampling_distance));
    for (int k = 1; k <= num_insert; ++k) {
      float t = (sampling_distance * k) / dist;
      result.push_back(interpolate_linear(p1, p2, t));
    }
  }

  result.push_back(polyline.back());
  return result;
}

void resample(const std::vector<std::vector<data_types::Point>> &input_polylines,
              std::vector<std::vector<data_types::Point>> &output_polylines,
              float sampling_distance) {
  for (const auto &polyline : input_polylines) {
    output_polylines.push_back(resample_single_polyline(polyline, sampling_distance));
  }
}

bool build(const PathManager::PathContext &ctx, PathManager::Stage eval_stage) {
  ROS_INFO("polyline evaluation start.");

  ros::NodeHandle nh("~");

  // evaluation parameters를 NodeHandle에서 가져옴
  float chamfer_dist_th, point_iou_th, resample_distance, dist_th, yaw_th;
  nh.param("chamfer_dist_th", chamfer_dist_th, 10.0f);
  nh.param("point_iou_th", point_iou_th, 0.5f);
  nh.param("resample_distance", resample_distance, 2.5f);
  nh.param("dist_th", dist_th, 0.5f);
  nh.param("yaw_th", yaw_th, 0.5f);

  bool partial_gt_mode;
  nh.param("partial_gt_mode", partial_gt_mode, false);

  const std::string draft_polyline_filename = PathManager::getStageBinaryPath(ctx, eval_stage, PathManager::BinaryKind::Polylines);
  const std::string gt_filename = PathManager::getGTBinaryPath(partial_gt_mode, ctx);
  const std::string draft_intersection_file_path = PathManager::getStageBinaryPath(ctx, eval_stage, PathManager::BinaryKind::DraftIntersections);
  const std::string gt_intersection_file_path = PathManager::getStageBinaryPath(ctx, eval_stage, PathManager::BinaryKind::GTIntersections);
  const std::string draft_fp_file_path = PathManager::getStageBinaryPath(ctx, eval_stage, PathManager::BinaryKind::DraftFPs);
  const std::string gt_fn_file_path = PathManager::getStageBinaryPath(ctx, eval_stage, PathManager::BinaryKind::GTFNs);

  // Step 1. Read input polyline file
  std::vector<std::vector<data_types::Point>> draft_polylines, draft_polylines_resampled, gt_polylines, gt_polylines_resampled;
  std::vector<std::vector<data_types::Point>> draft_intersections, gt_intersections, draft_false_positive, gt_false_negative;
  if (!io::load_polylines(draft_polyline_filename, draft_polylines)) {
    ROS_ERROR("Cannot open draft polyline file. (%s)", draft_polyline_filename.c_str());
    return false;
  }
  if (!io::load_polylines(gt_filename, gt_polylines)) {
    ROS_ERROR("Cannot open polyline gt file. (%s)", gt_filename.c_str());
    return false;
  }

  // Step 2. Resample polylines
  resample(draft_polylines, draft_polylines_resampled, resample_distance);
  resample(gt_polylines, gt_polylines_resampled, resample_distance);

  // Step 3. Calculate and print evaluation metric
  // todo: 검토 후 polyline_evaluator_coverage.cpp의 내용을 이곳으로 옮기고 AP / iou를 src_backup으로 이동
  // evaluate_AP(draft_polylines_resampled, gt_polylines_resampled, chamfer_dist_th);
  // evaluate_iou(draft_polylines_resampled, gt_polylines_resampled, point_iou_th);
  evaluate_len_coverage(draft_polylines_resampled, gt_polylines_resampled,
                        draft_intersections, gt_intersections, draft_false_positive, gt_false_negative,
                        dist_th, yaw_th);

  io::write_polylines(draft_intersection_file_path, draft_intersections);
  io::write_polylines(gt_intersection_file_path, gt_intersections);
  io::write_polylines(draft_fp_file_path, draft_false_positive);
  io::write_polylines(gt_fn_file_path, gt_false_negative);
  return true;
}
}  // namespace linemapdraft_builder::polyline_evaluator
