#pragma once

#include <array>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common/DataTypes.h"

class AccumulatedVoxelMap {
public:
    AccumulatedVoxelMap(float voxel_size, int yaw_voxel_num);
    ~AccumulatedVoxelMap() = default;

    void update(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    std::vector<VoxelPoint> buildVoxels() const;

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

    std::unordered_map<VoxelKey, VoxelAccum, ArrayHasher> voxel_map_;
    float voxel_size_;
    int yaw_voxel_num_;
};
