#include "real_time_map/GreedyLaneGenerator.h"
#include "toy_map_viewer/LaneUtils.h"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

GreedyLaneGenerator::GreedyLaneGenerator(const Config& config) : config_(config) {}

float GreedyLaneGenerator::getDist(const VoxelPoint& a, const VoxelPoint& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}

// 점 pt와 선분(line_start -> line_end) 사이의 수직 거리 계산
float GreedyLaneGenerator::getPerpendicularDist(const VoxelPoint& line_start, const VoxelPoint& line_end, const VoxelPoint& pt) {
    Eigen::Vector3f s(line_start.x, line_start.y, line_start.z);
    Eigen::Vector3f e(line_end.x, line_end.y, line_end.z);
    Eigen::Vector3f p(pt.x, pt.y, pt.z);

    Eigen::Vector3f line_vec = e - s;
    float line_len = line_vec.norm();
    if (line_len < 1e-6) return (p - s).norm();

    Eigen::Vector3f proj_vec = (p - s).cross(line_vec);
    return proj_vec.norm() / line_len;
}

void GreedyLaneGenerator::updateEWMA(float& ewma_dx, float& ewma_dy, float& ewma_vz, 
                                     const VoxelPoint& center, const VoxelPoint& next_pt) {
    // 현재 진행 방향 계산
    float dx = next_pt.x - center.x;
    float dy = next_pt.y - center.y;
    float dz = next_pt.z - center.z;
    float dist_xy = std::hypot(dx, dy);
    if(dist_xy < 1e-6) dist_xy = 1.0f;

    // Normalize direction
    float dir_x = dx / dist_xy;
    float dir_y = dy / dist_xy;
    float slope_z = dz / dist_xy;

    // 현재 점의 yaw 방향 벡터
    float pt_dir_x = std::cos(next_pt.yaw);
    float pt_dir_y = std::sin(next_pt.yaw);

    // Alpha, Beta 적용 (polyline_generator.cpp 참조)
    // ewma_yaw_x/y 는 코사인/사인 성분으로 관리
    ewma_dx = (1.0f - config_.alpha) * ewma_dx + config_.alpha * (config_.beta * dir_x + (1.0f - config_.beta) * pt_dir_x);
    ewma_dy = (1.0f - config_.alpha) * ewma_dy + config_.alpha * (config_.beta * dir_y + (1.0f - config_.beta) * pt_dir_y);
    
    // Z slope smoothing
    ewma_vz = (1.0f - config_.alpha) * ewma_vz + config_.alpha * slope_z;
}

std::map<int, Lane> GreedyLaneGenerator::generate(const std::vector<VoxelPoint>& voxels) {
    std::map<int, Lane> lanes;
    if (voxels.empty()) return lanes;

    // 1. 데이터 준비 (Nodes & KD-Tree)
    std::vector<Node> nodes;
    nodes.reserve(voxels.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    for (int i = 0; i < voxels.size(); ++i) {
        nodes.push_back({i, voxels[i], false, false});
        cloud->push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 2. 밀도 순 정렬 (Seeds)
    std::vector<int> sorted_indices(nodes.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return nodes[a].point.density > nodes[b].point.density;
    });

    int lane_id_counter = 0;

    // --- Lambda: 다음 점 찾기 (Cylinder Search & IQR Filtering) ---
    auto get_next_point = [&](const VoxelPoint& center, float smoothed_yaw, float search_width) -> int {
        // 예상 지점 (Hypothesis Point)
        pcl::PointXYZ search_pt;
        search_pt.x = center.x + config_.neighbor_dist_thresh * std::cos(smoothed_yaw);
        search_pt.y = center.y + config_.neighbor_dist_thresh * std::sin(smoothed_yaw);
        search_pt.z = center.z; // z는 일단 유지

        std::vector<int> idxs;
        std::vector<float> dists;
        
        // 탐색 반경은 neighbor_dist_thresh * 1.5 정도로 여유있게
        if (kdtree.radiusSearch(search_pt, config_.neighbor_dist_thresh * 1.2f, idxs, dists) <= 0) return -1;

        // 1차 필터링: Cylinder (거리 & 수직 거리)
        std::vector<int> candidates;
        VoxelPoint hp_pt = center; 
        hp_pt.x = search_pt.x; hp_pt.y = search_pt.y; hp_pt.z = search_pt.z;

        for (int idx : idxs) {
            if (nodes[idx].visited || nodes[idx].dropped) continue;
            
            // 현재 점과 후보 점 사이 거리 체크 (너무 멀면 제외)
            if (getDist(center, nodes[idx].point) > config_.neighbor_dist_thresh * 1.2f) continue;

            // 원기둥 내부 체크 (진행 방향 선분과의 수직 거리)
            float pd = getPerpendicularDist(center, hp_pt, nodes[idx].point);
            if (pd < search_width) {
                candidates.push_back(idx);
            }
        }

        if (candidates.empty()) return -1;

        // 2차 필터링: Yaw IQR
        if (candidates.size() > 2) {
            std::vector<float> yaws;
            for (int idx : candidates) yaws.push_back(nodes[idx].point.yaw);
            std::sort(yaws.begin(), yaws.end());
            float q1 = yaws[yaws.size() / 4];
            float q3 = yaws[yaws.size() * 3 / 4];
            float iqr = q3 - q1;
            float lower = q1 - config_.yaw_iqr_threshold * iqr;
            float upper = q3 + config_.yaw_iqr_threshold * iqr;

            // IQR 범위 내에서 가장 밀도가 높거나 중심에 가까운 점 선택
            // 여기서는 단순화를 위해 IQR 통과한 점 중 가장 가까운 점 선택 (혹은 평균)
            int best_idx = -1;
            float min_dist = std::numeric_limits<float>::max();

            for (int idx : candidates) {
                float y = nodes[idx].point.yaw;
                if (y >= lower && y <= upper) {
                    float d = getDist(center, nodes[idx].point);
                    if (d < min_dist) {
                        min_dist = d;
                        best_idx = idx;
                    }
                }
            }
            return best_idx;
        } else {
            // 후보가 적으면 그냥 가장 가까운 점
            return candidates[0]; 
        }
    };

    // --- Lambda: 주변 점 Drop (Sweep and Drop) ---
    auto sweep_and_drop = [&](const VoxelPoint& start, const VoxelPoint& end) {
        pcl::PointXYZ mid_pt((start.x + end.x)*0.5, (start.y + end.y)*0.5, (start.z + end.z)*0.5);
        std::vector<int> idxs;
        std::vector<float> dists;
        // 두 점 사이 거리만큼 반경 탐색
        float radius = getDist(start, end) * 0.6f + config_.drop_width; 
        
        if (kdtree.radiusSearch(mid_pt, radius, idxs, dists) > 0) {
            for (int idx : idxs) {
                if (nodes[idx].visited || nodes[idx].dropped) continue;
                float pd = getPerpendicularDist(start, end, nodes[idx].point);
                if (pd < config_.drop_width) {
                    nodes[idx].dropped = true; // Drop 처리
                }
            }
        }
    };

    // --- Lambda: 단방향 확장 ---
    auto extend_lane = [&](int start_idx, bool reverse) -> std::vector<Point6D> {
        std::vector<Point6D> lane_pts;
        
        // 시작점 정보
        VoxelPoint curr_pt = nodes[start_idx].point;
        float curr_yaw = curr_pt.yaw;
        if (reverse) curr_yaw += M_PI; // 반대 방향이면 Yaw 뒤집기

        // EWMA 초기화
        float ewma_dx = std::cos(curr_yaw);
        float ewma_dy = std::sin(curr_yaw);
        float ewma_vz = 0.0f;

        // 시작점 추가
        Point6D start_p6;
        start_p6.x = curr_pt.x; start_p6.y = curr_pt.y; start_p6.z = curr_pt.z;
        start_p6.dx = ewma_dx; start_p6.dy = ewma_dy; start_p6.dz = ewma_vz;
        lane_pts.push_back(start_p6);

        int prev_node_idx = start_idx;

        while(true) {
            float smoothed_yaw = std::atan2(ewma_dy, ewma_dx);
            
            // 다음 점 찾기
            int next_idx = get_next_point(curr_pt, smoothed_yaw, config_.cylinder_search_width);
            if (next_idx == -1) break;

            // 상태 업데이트
            nodes[next_idx].visited = true;
            VoxelPoint next_vp = nodes[next_idx].point;
            
            // Sweep and Drop
            sweep_and_drop(curr_pt, next_vp);

            // EWMA 업데이트
            if (reverse) next_vp.yaw += M_PI; // 방향 보정 후 EWMA 계산
            updateEWMA(ewma_dx, ewma_dy, ewma_vz, curr_pt, next_vp);

            // 결과 저장
            Point6D p6;
            p6.x = next_vp.x; p6.y = next_vp.y; p6.z = next_vp.z;
            p6.dx = std::cos(std::atan2(ewma_dy, ewma_dx)); // Smoothed dir
            p6.dy = std::sin(std::atan2(ewma_dy, ewma_dx));
            p6.dz = ewma_vz;
            lane_pts.push_back(p6);

            // 이동
            curr_pt = next_vp;
            prev_node_idx = next_idx;
        }
        return lane_pts;
    };

    // 3. 메인 루프
    for (int idx : sorted_indices) {
        if (nodes[idx].visited || nodes[idx].dropped) continue;
        if (nodes[idx].point.density < (uint32_t)config_.min_density) continue;

        nodes[idx].visited = true;

        // 양방향 확장
        std::vector<Point6D> forward = extend_lane(idx, false);   // 정방향
        std::vector<Point6D> backward = extend_lane(idx, true);   // 역방향

        // 하나로 합치기: backward(역순) + forward
        std::vector<Point6D> merged;
        merged.insert(merged.end(), backward.rbegin(), backward.rend());
        // forward의 첫 점은 시드점(backward의 첫 점)과 같으므로 중복 제거
        if (!forward.empty()) {
            merged.insert(merged.end(), forward.begin() + 1, forward.end());
        } else {
             // Backward만 있고 Forward가 실패했을 경우 시드점이 backward에 포함되어 있음
        }

        if (merged.size() >= 3) { // 최소 점 개수 제한
            Lane lane;
            lane.id = lane_id_counter++;
            lane.points = merged;
            lane.explicit_lane = true;
            lane.valid = true;
            
            // 방향 재정렬 및 정리
            LaneUtils::ReorderPoints(lane);
            lanes[lane.id] = lane;
        }
    }

    return lanes;
}