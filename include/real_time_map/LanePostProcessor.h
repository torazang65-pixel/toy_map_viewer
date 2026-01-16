#pragma once

#include <vector>
#include <map>
#include <optional>
#include "common/DataTypes.h"

class LanePostProcessor {
public:
    struct Config {
        // post_processor.cpp의 파라미터 대응
        float merge_min_dist_th = 1.0f;
        float merge_max_dist_th = 10.0f;
        float merge_min_angle_th = 0.1f; // rad
        float merge_max_angle_th = 0.3f; // rad
        
        // 필터링 옵션 (선택)
        float min_lane_length = 3.0f; 
    };

    LanePostProcessor(const Config& config);
    ~LanePostProcessor() = default;

    // LaneClusterer와 동일한 인터페이스
    std::vector<Lane> clusterAndSort(const std::map<int, Lane>& fragment_lanes);

private:
    struct MergeCandidate {
        int lane_id_1;
        int lane_id_2;
        int point_idx_1; // 0 (start) or size-1 (end)
        int point_idx_2; // 0 (start) or size-1 (end)
        float cost;

        bool operator<(const MergeCandidate& other) const {
            return cost < other.cost;
        }
    };

    // 내부 헬퍼 함수 (post_processor.cpp 로직 이식)
    std::optional<float> getMergeCost(const Point6D& end1, const Point6D& prev1,
                                      const Point6D& end2, const Point6D& prev2);
    
    std::optional<MergeCandidate> checkMerge(const Lane& l1, const Lane& l2);

    Lane mergeLanes(const Lane& l1, const Lane& l2, int idx1, int idx2);

    Config config_;
    int merged_lane_id_counter_ = 0;
};