#pragma once

#include <common/data_types.h>

struct LaneConfig {
    //Map Convereter Node
    double crop_size = 300.0;
    int file_idx = 20000;
    int end_index = 21000;
    int load_count = 1;
    bool crop_mode = true;
    bool random_index = false;

    //Lane Cleaner
    double overlap_radius = 0.3; // meters
    double linearity_tolerance = 0.02;

    //Lane Merger
    double search_radius = 1.5; // meters
    double direction_threshold_deg = 30.0; // degrees
    double weight_distance = 1.0;
    double weight_direction = 5.0;
};

namespace LaneUtils{
    // Math Utils
    double GetDistanceSq(const linemapdraft_builder::data_types::Point6D& a,
                         const linemapdraft_builder::data_types::Point6D& b);
    double GetDistance(const linemapdraft_builder::data_types::Point6D& a,
                       const linemapdraft_builder::data_types::Point6D& b);
    double GetAngleDegrees(const linemapdraft_builder::data_types::Point6D& v1,
                           const linemapdraft_builder::data_types::Point6D& v2);

    // Lane Utils
    /**
     * @brief calculate length of a lane
     */
    double CalculateLaneLength(const linemapdraft_builder::data_types::Lane& lane);
    /**
     * @brief calculate linearity of a lane
     */
    double CalculateLaneLinearity(const linemapdraft_builder::data_types::Lane& lane);

    /**
     * @brief reorder Lane base on distance and direction of points (legacy version)
     */
    void ReorderPoints(linemapdraft_builder::data_types::Lane& lane);

    /**
     * @brief Improved reorder Lane using KD-tree and PCA-based endpoint detection
     * @param lane Lane to reorder
     * @param smooth Enable smoothing pass (default: true)
     * @param maxPointDistance Maximum allowed distance between consecutive points (default: 3.0m)
     */
    void ReorderPointsImproved(linemapdraft_builder::data_types::Lane& lane);
    // Can add new functions here about lanes (i.e. distance between lanes, etc.)
    // double CalculateLaneLength(const Lane& lane);
}
