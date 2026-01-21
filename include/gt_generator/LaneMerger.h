#pragma once

#include <map>
#include <common/data_types.h>
#include "LaneUtils.h"

namespace LaneMerger {
void MergeFragmentedLanes(std::map<int, linemapdraft_builder::data_types::Lane>& global_map,
                          const LaneConfig& config);
} // namespace LaneMerger
