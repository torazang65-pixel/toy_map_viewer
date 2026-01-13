#include "real_time_map/VoxelBuilder.h"
#include <cmath>
#include <algorithm>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

VoxelBuilder::VoxelBuilder(float voxel_size, int yaw_voxel_num)
    : voxel_size_(voxel_size), yaw_voxel_num_(yaw_voxel_num){}

std::vector<VoxelPoint> VoxelBuilder::build(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (cloud->empty()) return {};

    std::unordered_map<VoxelKey, VoxelAccum, ArrayHasher> voxel_map;
    float yaw_voxel_size = M_PI / yaw_voxel_num_;

    // 1. Accumulate
    for (const auto& pt : cloud->points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        float yaw = pt.intensity; // Intensity 필드에 Yaw 저장됨

        int vx = static_cast<int>(std::floor(x / voxel_size_));
        int vy = static_cast<int>(std::floor(y / voxel_size_));
        int vz = static_cast<int>(std::floor(z / voxel_size_));
        int vyaw = static_cast<int>(std::floor(yaw / yaw_voxel_size));

        VoxelKey key = {vx, vy, vz, vyaw};
        auto& acc = voxel_map[key];
        acc.count++;
        acc.sum_x += x;
        acc.sum_y += y;
        acc.sum_z += z;
        acc.sum_yaw += yaw;
    }

    // 2. Calculate Threshold (Q1 Filtering)
    std::vector<uint32_t> counts;
    counts.reserve(voxel_map.size());
    for (const auto& pair : voxel_map) {
        if (pair.second.count > 1) {
            counts.push_back(pair.second.count);
        }
    }
    
    uint32_t threshold_count = 0;
    if (!counts.empty()) {
        std::sort(counts.begin(), counts.end());
        threshold_count = counts[counts.size() / 4];
    }

    // 3. Generate Voxel Points
    std::vector<VoxelPoint> result;
    result.reserve(voxel_map.size());

    for (const auto& pair : voxel_map) {
        const auto& acc = pair.second;
        if (acc.count <= threshold_count) continue;

        VoxelPoint vp;
        vp.x = static_cast<float>(acc.sum_x / acc.count);
        vp.y = static_cast<float>(acc.sum_y / acc.count);
        vp.z = static_cast<float>(acc.sum_z / acc.count);
        vp.yaw = static_cast<float>(acc.sum_yaw / acc.count);
        vp.density = acc.count;

        // Yaw Normalize
        while (vp.yaw < 0) vp.yaw += 2 * M_PI;
        while (vp.yaw >= M_PI) vp.yaw -= M_PI;

        result.push_back(vp);
    }

    return result;
}