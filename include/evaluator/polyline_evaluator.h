#pragma once

#include <common/path_manager.h>

#include <string>

namespace linemapdraft_builder::polyline_evaluator {
void evaluate_len_coverage(const std::vector<std::vector<data_types::Point>> &preds,
                           const std::vector<std::vector<data_types::Point>> &gts,
                           std::vector<std::vector<data_types::Point>> &draft_intersections,
                           std::vector<std::vector<data_types::Point>> &gt_intersections,
                           std::vector<std::vector<data_types::Point>> &draft_fps,
                           std::vector<std::vector<data_types::Point>> &gt_fns,
                           float dist_th,
                           float yaw_th);

bool build(const PathManager::PathContext &ctx, PathManager::Stage eval_stage);

// [추가] 리샘플링 함수 선언
void resample(const std::vector<std::vector<data_types::Point>> &input_polylines,
              std::vector<std::vector<data_types::Point>> &output_polylines,
              float sampling_distance);

// [추가] 폴리라인 전체 길이 계산 함수 선언
double total_length(const std::vector<std::vector<data_types::Point>> &polylines);

}  // namespace linemapdraft_builder::polyline_evaluator
