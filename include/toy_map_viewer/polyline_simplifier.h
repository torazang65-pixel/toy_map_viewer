#pragma once

#include <common/path_manager.h>

#include <string>

namespace linemapdraft_builder::polyline_simplifier {

bool build(const PathManager::PathContext &ctx,
           const PathManager::Stage prev_stage,
           const PathManager::Stage cur_stage,
           const bool vw_flag,
           const bool rdp_flag);
}
