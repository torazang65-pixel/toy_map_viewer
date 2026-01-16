#pragma once

#include <vector>
#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "common/DataTypes.h"

class PCALaneGenerator {
public:
    struct Config {
        float search_radius = 3.0f;      // RANSAC 후보군 탐색 반경
        float yaw_threshold = 30.0f;     // Yaw 허용 오차 (도)
        int min_inliers = 5;             // 라인으로 인정할 최소 점 개수
        float distance_threshold = 0.2f; // RANSAC 거리 허용 오차 (m)
        int min_density = 5;             // 시드 점 최소 밀도
        float z_tolerance = 1.0f;
        float max_lane_length = 50.0f;   // 최대 lane 길이 (m) - 한 방향 확장 거리
    };

    PCALaneGenerator(const Config& config);
    ~PCALaneGenerator() = default;

    // VoxelPoints를 받아 Lane 맵을 반환
    std::map<int, Lane> generate(const std::vector<VoxelPoint>& voxels);

private:
    // 내부 헬퍼 구조체
    struct VoxelNode {
        int id;
        VoxelPoint point;
        bool visited = false;
    };

    void fitLocalLine(const std::vector<VoxelNode>& candidates, 
                      std::vector<Point6D>& out_points, 
                      std::vector<int>& out_inlier_indices);

    Config config_;
};
