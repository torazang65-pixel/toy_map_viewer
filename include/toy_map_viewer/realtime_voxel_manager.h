#ifndef TOY_MAP_VIEWER_REALTIME_VOXEL_MANAGER_H
#define TOY_MAP_VIEWER_REALTIME_VOXEL_MANAGER_H

#include <unordered_map>
#include <vector>
#include <array>
#include <common/data_types.h>

namespace toy_map_viewer {

namespace ldb = linemapdraft_builder;

using VoxelKey = std::array<int, 4>;

struct VoxelAccum {
    uint32_t count = 0;
    double sum_x = 0, sum_y = 0, sum_z = 0, sum_yaw = 0;
    uint32_t last_frame_id = 0;
};

struct ArrayHasher {
    std::size_t operator()(const std::array<int, 4> &key) const {
        return std::hash<int>()(key[0]) * 73856093 ^
               std::hash<int>()(key[1]) * 19349663 ^
               std::hash<int>()(key[2]) * 83492791 ^
               std::hash<int>()(key[3]) * 2971215073;
    }
};

class RealTimeVoxelManager {
public:
    RealTimeVoxelManager(float voxel_size, int yaw_num);
    
    // 신규 프레임 점들을 기존 보셀 데이터에 누적
    void accumulate(const std::vector<ldb::data_types::Point>& frame_points, uint32_t frame_id);
    
    // 밀도 조건을 만족하는 보셀만 포인트로 추출
    std::vector<ldb::data_types::Point> getFilteredPoints();
    
    // 오래된 보셀을 삭제하여 메모리 관리
    void clearStaleVoxels(uint32_t current_frame_id, uint32_t max_age);

private:
    std::unordered_map<VoxelKey, VoxelAccum, ArrayHasher> voxel_map_;
    float voxel_size_;
    int yaw_voxel_num_;
};

} // namespace toy_map_viewer

#endif