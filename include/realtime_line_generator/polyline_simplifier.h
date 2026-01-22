#pragma once

#include <common/path_manager.h>

#include <string>

namespace linemapdraft_builder::polyline_simplifier {

bool build(const PathManager::PathContext &ctx,
           const PathManager::Stage prev_stage,
           const PathManager::Stage cur_stage,
           const bool vw_flag,
           const bool rdp_flag);


// [추가] VW(Visvalingam-Whyatt) 단순화 함수 선언
void vw(std::vector<data_types::Point>& polyline, float area_thresh);

// [추가] RDP(Ramer-Douglas-Peucker) 단순화 함수 선언
void rdp(std::vector<data_types::Point>& polyline, float eps);

}
