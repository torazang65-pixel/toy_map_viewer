#pragma once
#include <common/path_manager.h>
#include <common/data_types.h> // Point 및 GridMap 정의를 위해 필요
#include <string>
#include <vector>

namespace linemapdraft_builder::polyline_generator {

bool build(const PathManager::PathContext &ctx);

// 실시간용으로 노출할 함수 선언 추가
void generate_polyline(std::vector<data_types::Point>& points,
                       std::vector<std::vector<data_types::Point>>& polylines,
                       const data_types::GridMap& grid,
                       const float grid_size,
                       const uint32_t min_density,
                       const float neighbor_dist_thresh,
                       const float cylinder_search_width,
                       const float alpha,
                       const float beta,
                       const float drop_width);

}