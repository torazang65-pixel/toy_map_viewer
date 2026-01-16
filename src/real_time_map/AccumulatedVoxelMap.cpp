#include "real_time_map/AccumulatedVoxelMap.h"

#include <algorithm>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AccumulatedVoxelMap::AccumulatedVoxelMap(float voxel_size, int yaw_voxel_num)
    : voxel_size_(voxel_size), yaw_voxel_num_(yaw_voxel_num) {}

void AccumulatedVoxelMap::update(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (!cloud || cloud->empty()) return;

    float yaw_voxel_size = M_PI / yaw_voxel_num_;
    for (const auto& pt : cloud->points) {
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        float yaw = pt.intensity; // intensity에 yaw 저장

        int vx = static_cast<int>(std::floor(x / voxel_size_));
        int vy = static_cast<int>(std::floor(y / voxel_size_));
        int vz = static_cast<int>(std::floor(z / voxel_size_));
        int vyaw = static_cast<int>(std::floor(yaw / yaw_voxel_size));

        VoxelKey key = {vx, vy, vz, vyaw};
        auto& acc = voxel_map_[key];
        acc.count++;
        acc.sum_x += x;
        acc.sum_y += y;
        acc.sum_z += z;
        acc.sum_yaw += yaw;
    }
}

std::vector<VoxelPoint> AccumulatedVoxelMap::buildVoxels() const {
    if (voxel_map_.empty()) return {};

    std::vector<uint32_t> counts;
    counts.reserve(voxel_map_.size());
    for (const auto& pair : voxel_map_) {
        if (pair.second.count > 1) {
            counts.push_back(pair.second.count);
        }
    }

    uint32_t threshold_count = 0;
    if (!counts.empty()) {
        std::sort(counts.begin(), counts.end());
        threshold_count = counts[counts.size() / 4];
    }

    std::vector<VoxelPoint> result;
    result.reserve(voxel_map_.size());

    for (const auto& pair : voxel_map_) {
        const auto& acc = pair.second;
        if (acc.count <= static_cast<int>(threshold_count)) continue;

        VoxelPoint vp;
        vp.x = static_cast<float>(acc.sum_x / acc.count);
        vp.y = static_cast<float>(acc.sum_y / acc.count);
        vp.z = static_cast<float>(acc.sum_z / acc.count);
        vp.yaw = static_cast<float>(acc.sum_yaw / acc.count);
        vp.density = acc.count;

        while (vp.yaw < 0) vp.yaw += 2 * M_PI;
        while (vp.yaw >= M_PI) vp.yaw -= M_PI;

        result.push_back(vp);
    }

    return result;
}
