#pragma once

#include <common/path_manager.h>

#include <string>

namespace linemapdraft_builder::post_processor {

bool build(const PathManager::PathContext &ctx);

// 실시간용으로 노출할 함수들 추가
std::vector<std::vector<data_types::Point>> mergeAllPolylines(
    const std::vector<std::vector<data_types::Point>>& original_polylines,
    const float merge_min_dist_th,
    const float merge_max_dist_th,
    const float merge_min_angle_th,
    const float merge_max_angle_th);

void dropPolylines(
    std::vector<std::vector<data_types::Point>>& polylines,
    const float min_polyline_length);

}