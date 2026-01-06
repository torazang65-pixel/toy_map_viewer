#pragma once

#include <map>
#include "DataTypes.h"
#include "LaneUtils.h"

namespace LaneMerger {
void MergeFragmentedLanes(std::map<int, Lane>& global_map, const LaneConfig& config);
} // namespace LaneMerger