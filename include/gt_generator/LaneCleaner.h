#pragma once

#include <map>
#include <common/data_types.h>
#include "LaneUtils.h"

namespace LaneCleaner {
    /**
     * @brief Trim overlapping lane ends based on proximity and length
     */
    void TrimOverlappingLaneEnds(
        std::map<int, linemapdraft_builder::data_types::Lane>& global_map,
        const LaneConfig& config);
} 
