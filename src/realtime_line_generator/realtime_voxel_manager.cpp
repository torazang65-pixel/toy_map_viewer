#include <realtime_line_generator/realtime_voxel_manager.h>
#include <common/data_types.h>
#include <cmath>

namespace realtime_line_generator {

namespace ldb = linemapdraft_builder;

RealTimeVoxelManager::RealTimeVoxelManager(float voxel_size, int yaw_num) 
    : voxel_size_(voxel_size), yaw_voxel_num_(yaw_num) {}

void RealTimeVoxelManager::accumulate(const std::vector<ldb::data_types::Point>& frame_points, uint32_t frame_id) {
    const float yaw_voxel_size = M_PI / yaw_voxel_num_;

    for (const auto& pt : frame_points) {
        int vx = static_cast<int>(std::floor(pt.x / voxel_size_));
        int vy = static_cast<int>(std::floor(pt.y / voxel_size_));
        int vz = static_cast<int>(static_cast<float>(pt.z) / voxel_size_); // z축 보셀화
        int vyaw = static_cast<int>(std::floor(pt.yaw / yaw_voxel_size));

        VoxelKey key = {vx, vy, vz, vyaw};
        auto& acc = voxel_map_[key];
        
        acc.count++;
        acc.sum_x += pt.x;
        acc.sum_y += pt.y;
        acc.sum_z += pt.z;
        acc.sum_yaw += pt.yaw;
        acc.last_frame_id = frame_id;
    }
}

std::vector<ldb::data_types::Point> RealTimeVoxelManager::getFilteredPoints(uint32_t min_density_threshold) {
    std::vector<ldb::data_types::Point> filtered_points;

    for (const auto& [key, acc] : voxel_map_) {
        if (min_density_threshold > 0 && acc.count < min_density_threshold) {
            continue;
        }

        float x = acc.sum_x / acc.count;
        float y = acc.sum_y / acc.count;
        float z = acc.sum_z / acc.count;
        float yaw = acc.sum_yaw / acc.count;

        // [기존 로직 그대로] Yaw 정규화 (0~PI)
        while (yaw < 0) yaw += 2 * M_PI;
        while (yaw >= M_PI) yaw -= M_PI;

        linemapdraft_builder::data_types::Point p;
        p.x = x; p.y = y; p.z = z; p.yaw = yaw;
        p.density = acc.count;
        filtered_points.push_back(p);
    }
    return filtered_points;
}

void RealTimeVoxelManager::clearStaleVoxels(uint32_t current_id, uint32_t max_age) {
    // 차량 이동에 따른 오래된 데이터 삭제 (Aging 로직)
    for (auto it = voxel_map_.begin(); it != voxel_map_.end(); ) {
        if (current_id - it->second.last_frame_id > max_age) {
            it = voxel_map_.erase(it);
        } else {
            ++it;
        }
    }
}

}
