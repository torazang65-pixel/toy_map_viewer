#pragma once

#include <vector>

#include <common/data_types.h>

namespace realtime_line_generator::viewer {

struct OffsetState {
    double x = 0.0;
    double y = 0.0;
    bool initialized = false;
};

inline void MaybeInitOffsetFromPoints(
    OffsetState& offset,
    const std::vector<linemapdraft_builder::data_types::Point>& points) {
    if (offset.initialized || points.empty()) {
        return;
    }
    offset.x = points.front().x;
    offset.y = points.front().y;
    offset.initialized = true;
}

inline void MaybeInitOffsetFromPolylines(
    OffsetState& offset,
    const std::vector<std::vector<linemapdraft_builder::data_types::Point>>& polylines) {
    if (offset.initialized) {
        return;
    }
    if (polylines.empty() || polylines.front().empty()) {
        return;
    }
    offset.x = polylines.front().front().x;
    offset.y = polylines.front().front().y;
    offset.initialized = true;
}

}  // namespace realtime_line_generator::viewer
