#pragma once

#include <map>
#include "common/DataTypes.h"
#include "LaneUtils.h"

namespace LaneCleaner {
    /**
     * @brief Trim overlapping lane ends based on proximity and length
     */
    void TrimOverlappingLaneEnds(std::map<int, Lane>& global_map, const LaneConfig& config);
}