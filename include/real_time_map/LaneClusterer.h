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
        // [Step 1] Segment Clustering 파라미터 (LaneMerger와 유사)
        double merge_search_radius = 2.5;       // Segment 연결 탐색 반경 (m)
        double merge_angle_threshold = 30.0;    // 연결 허용 각도 (도)
        
        // [Step 2] Geodesic Sort 파라미터
        double point_connection_radius = 0.8;   // 점 단위 그래프 연결 반경 (m)
        // 이 반경이 너무 크면 U턴 구간이 가로질러 연결될 수 있고, 너무 작으면 끊어질 수 있음.
        // voxel_size(0.5)보다 약간 크게 설정 (예: 0.8 ~ 1.0)
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
    // --- 내부 자료구조 ---
    struct GraphNode {
        int id;
        std::vector<std::pair<int, double>> neighbors; // {neighbor_idx, weight(dist)}
    };

    // --- Helper Functions: Step 1. Clustering ---
    // 세그먼트 간의 연결 그래프 생성
    void buildSegmentGraph(const std::vector<Lane*>& lanes, 
                           std::vector<std::vector<int>>& adj_list);
    
    // 두 세그먼트가 연결 가능한지 판단 (거리 & 방향)
    bool isConnectable(const Lane& l1, const Lane& l2);

    // --- Helper Functions: Step 2. Sorting ---
    // 하나의 Lane 덩어리 내부 점들을 Geodesic Distance 기준으로 정렬
    void sortPointsGeodesically(Lane& lane);

    // 점들을 그래프로 변환 (KD-Tree 사용)
    std::vector<GraphNode> buildPointGraph(const std::vector<Point6D>& points);

    // 그래프의 지름(Diameter)을 구성하는 양 끝점 찾기 (Two-pass Dijkstra)
    // first: start_node_idx, second: end_node_idx
    std::pair<int, int> findGraphEndpoints(const std::vector<GraphNode>& graph);

    // 특정 시작점으로부터 모든 점까지의 최단 경로(Geodesic Distance) 계산
    std::vector<double> computeDistances(const std::vector<GraphNode>& graph, int start_node);

    Config config_;
    int merged_lane_id_counter_ = 0;
};