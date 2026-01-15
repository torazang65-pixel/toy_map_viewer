#pragma once

#include <vector>
#include <array>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "common/DataTypes.h"

class VoxelBuilder {
public:
    VoxelBuilder(float voxel_size, int yaw_voxel_num);
    ~VoxelBuilder() = default;

    // 포인트 클라우드를 입력받아 Voxelization 결과 반환
    std::vector<VoxelPoint> build(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

private:
    struct VoxelAccum {
        int count = 0;
        double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
        double sum_yaw = 0.0;
    };

    struct ArrayHasher {
        std::size_t operator()(const std::array<int, 4>& key) const {
            return std::hash<int>()(key[0]) * 73856093 ^
                   std::hash<int>()(key[1]) * 19349663 ^
                   std::hash<int>()(key[2]) * 83492791 ^
                   std::hash<int>()(key[3]) * 2971215073;
        }
    };

    using VoxelKey = std::array<int, 4>; // vx, vy, vz, vyaw

    float voxel_size_;
    int yaw_voxel_num_;
};