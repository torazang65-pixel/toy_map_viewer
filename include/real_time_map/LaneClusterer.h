#pragma once

#include <vector>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "common/DataTypes.h"

class LaneClusterer {
public:
    struct Config {
        // Segment Clustering 파라미터
        double merge_search_radius = 2.5;       // Segment 연결 탐색 반경 (m)
        double merge_angle_threshold = 30.0;    // 연결 허용 각도 (도) - 최대값
        double merge_min_angle_threshold = 5.0; // 최소 각도 허용 범위 (가까운 거리용, 도)
        double merge_min_dist_for_angle = 0.5;  // 최소 각도를 적용할 거리 (m)
        double min_lane_length = 3.0;           // 최소 차선 길이 (m)
    };

    LaneClusterer(const Config& config);
    ~LaneClusterer() = default;

    /**
     * @brief 파편화된 Lane들을 입력받아 클러스터링 및 정렬 후 반환
     * @param fragment_lanes Ransac/Greedy 등으로 생성된 조각난 Lane들
     * @return 병합되고 정렬된 Lane 리스트
     */
    std::vector<Lane> clusterAndSort(const std::map<int, Lane>& fragment_lanes);

private:
    // --- Helper Functions: Segment Clustering ---
    // 세그먼트 간의 연결 그래프 생성
    void buildSegmentGraph(const std::vector<Lane*>& lanes,
                           std::vector<std::vector<int>>& adj_list);

    // 두 세그먼트가 연결 가능한지 판단 (거리 & 방향)
    bool isConnectable(const Lane& l1, const Lane& l2);

    Config config_;
    int merged_lane_id_counter_ = 0;
};