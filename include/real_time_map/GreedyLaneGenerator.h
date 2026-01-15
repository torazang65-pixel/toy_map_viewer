#pragma once

#include <vector>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "common/DataTypes.h"

class GreedyLaneGenerator {
public:
    struct Config {
        float neighbor_dist_thresh = 2.0f;    // 다음 점 탐색 반경
        float cylinder_search_width = 1.0f;   // 원기둥 탐색 너비
        float alpha = 0.7f;                   // EWMA weight (위치)
        float beta = 0.9f;                    // EWMA weight (방향)
        float drop_width = 1.0f;              // 생성된 라인 주변 점 제거 폭
        int min_density = 5;                  // 시드 점 최소 밀도
        float yaw_iqr_threshold = 1.5f;       // Yaw 이상치 제거용 IQR multiplier
    };

    GreedyLaneGenerator(const Config& config);
    ~GreedyLaneGenerator() = default;

    // VoxelPoints를 받아 Lane 맵을 반환 (RansacLaneGenerator와 동일 인터페이스)
    std::map<int, Lane> generate(const std::vector<VoxelPoint>& voxels);

private:
    // 내부 연산용 구조체
    struct Node {
        int id;
        VoxelPoint point;
        bool visited = false;
        bool dropped = false; // sweepAndDrop에 의해 제거된 경우
    };

    // Helper functions mapping logic from polyline_generator.cpp
    float getDist(const VoxelPoint& a, const VoxelPoint& b);
    float getPerpendicularDist(const VoxelPoint& line_start, const VoxelPoint& line_end, const VoxelPoint& pt);
    
    // EWMA 업데이트
    void updateEWMA(float& ewma_dx, float& ewma_dy, float& ewma_vz, 
                    const VoxelPoint& center, const VoxelPoint& next_pt);

    Config config_;
};