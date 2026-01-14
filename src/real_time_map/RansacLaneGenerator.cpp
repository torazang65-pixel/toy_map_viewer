#include "real_time_map/RansacLaneGenerator.h"
#include "toy_map_viewer/LaneUtils.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <cmath>
#include <iostream>

RansacLaneGenerator::RansacLaneGenerator(const Config& config) : config_(config) {}

std::map<int, Lane> RansacLaneGenerator::generate(const std::vector<VoxelPoint>& voxels) {
    std::map<int, Lane> lanes;
    if (voxels.empty()) return lanes;

    // 1. 데이터 준비 및 KD-Tree 구성
    std::vector<VoxelNode> nodes;
    nodes.reserve(voxels.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < voxels.size(); ++i) {
        nodes.push_back({i, voxels[i], false}); // visited = false
        cloud->push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 2. 시드 정렬 (밀도 순, 기존과 동일)
    std::vector<int> sorted_indices(nodes.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return nodes[a].point.density > nodes[b].point.density;
    });

    int lane_id_counter = 0;
    float rad_threshold = config_.yaw_threshold * M_PI / 180.0f;

    // lambda function
    auto expand_lane = [&](int start_node_idx, bool is_forward) -> std::vector<Point6D> {
        std::vector<Point6D> segment_points;
        VoxelNode* current_node = &nodes[start_node_idx];
        
        // 후방 확장의 경우, 시드점의 Yaw와 반대 방향(180도)을 바라봐야 함
        // 하지만 '점의 방향(Orientation)' 자체는 차선 진행 방향과 같아야 하므로
        // Spatial Check에서만 각도를 뒤집어서 계산함.

        while (true) {
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            pcl::PointXYZ search_pt(current_node->point.x, current_node->point.y, current_node->point.z);
            
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) break;

            std::vector<VoxelNode> ransac_candidates;
            std::vector<int> candidate_global_indices;
            float current_yaw = current_node->point.yaw;

            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue;

                // A. Orientation Consistency (점 자체의 방향이 차선 방향과 일치하는가?)
                float yaw_diff = std::abs(nodes[ni].point.yaw - current_yaw);
                while(yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                if (std::abs(yaw_diff) > rad_threshold) continue;

                // B. Spatial Alignment Check (위치가 앞/뒤에 있는가?)
                float dx = nodes[ni].point.x - current_node->point.x;
                float dy = nodes[ni].point.y - current_node->point.y;
                float conn_yaw = std::atan2(dy, dx);

                // 기준 각도: 전방이면 current_yaw, 후방이면 current_yaw + PI
                float target_spatial_yaw = is_forward ? current_yaw : (current_yaw + M_PI);
                
                float spatial_diff = target_spatial_yaw - conn_yaw;
                while(spatial_diff > M_PI) spatial_diff -= 2 * M_PI;
                while(spatial_diff < -M_PI) spatial_diff += 2 * M_PI;
                
                // 지정된 방향(앞 또는 뒤)의 부채꼴 안에 들어오는지 확인
                if (std::abs(spatial_diff) > rad_threshold) continue;

                ransac_candidates.push_back(nodes[ni]);
                candidate_global_indices.push_back(ni);
            }

            if (ransac_candidates.size() < config_.min_inliers) break;

            // 로컬 피팅 수행
            std::vector<Point6D> fitted_segment;
            std::vector<int> inlier_local_indices;
            fitLocalLine(ransac_candidates, fitted_segment, inlier_local_indices); // fitted_segment에는 dx,dy,dz가 포함됨

            if (fitted_segment.empty()) break;

            // Inlier 방문 처리
            for (int local_idx : inlier_local_indices) {
                nodes[candidate_global_indices[local_idx]].visited = true;
            }

            // 결과 저장
            // fitLocalLine은 항상 [t_min -> t_max] 순서로 점을 반환 (차선 진행 방향 순)
            // 전방 확장이면: 뒤에 붙임
            // 후방 확장이면: 앞에 붙임 (나중에 처리) -> 일단 순서대로 모으고 나중에 합침
            segment_points.insert(segment_points.end(), fitted_segment.begin(), fitted_segment.end());

            // 다음 시드 업데이트
            // 전방 확장: 가장 멀리 있는 점(t_max)에 해당하는 Voxel 찾기
            // 후방 확장: 가장 뒤에 있는 점(t_min)에 해당하는 Voxel 찾기
            // (간단히 구현하기 위해 Inlier 중 마지막/첫번째 인덱스 사용)
            // fitLocalLine 내부 구현에 따라 inliers 순서가 보장되지 않을 수 있으므로 주의 필요
            // 여기서는 fitted_segment의 끝점 좌표와 가장 가까운 candidate를 찾는 방식 추천
            
            Point6D next_seed_pt;
            if (is_forward) next_seed_pt = fitted_segment.back(); // 전방의 끝
            else next_seed_pt = fitted_segment.front();           // 후방의 끝(기하학적으로 가장 뒤)

            // 다음 시드(Voxel) 찾기 - 가장 가까운 candidate 선택
            int best_cand_idx = -1;
            float min_dist_sq = std::numeric_limits<float>::max();
            for (int idx : candidate_global_indices) {
                float d2 = std::pow(nodes[idx].point.x - next_seed_pt.x, 2) + 
                           std::pow(nodes[idx].point.y - next_seed_pt.y, 2);
                if (d2 < min_dist_sq) {
                    min_dist_sq = d2;
                    best_cand_idx = idx;
                }
            }
            
            if (best_cand_idx != -1) {
                current_node = &nodes[best_cand_idx];
            } else {
                break; // 다음 시드를 못 찾음
            }
        }
        return segment_points;
    };

    // 3. 메인 루프 (Seed 순회)
    for (int idx : sorted_indices) {
        if (nodes[idx].visited) continue;
        if (nodes[idx].point.density < config_.min_density) continue;

        nodes[idx].visited = true; // 시드 방문 처리

        // 양방향 확장 수행
        std::vector<Point6D> forward_pts = expand_lane(idx, true);
        std::vector<Point6D> backward_pts = expand_lane(idx, false);
        
        // 모든 점 합치기
        std::vector<Point6D> all_points;
        all_points.insert(all_points.end(), backward_pts.begin(), backward_pts.end());

        Point6D seed_p; 
        seed_p.x = nodes[idx].point.x; seed_p.y = nodes[idx].point.y; seed_p.z = nodes[idx].point.z;
        // seed_p의 dx,dy,dz는 구하기 애매하므로 생략하거나 forward_pts의 첫 방향 복사
        if(!forward_pts.empty()) { seed_p.dx = forward_pts[0].dx; seed_p.dy = forward_pts[0].dy; seed_p.dz = forward_pts[0].dz; }
        else if(!backward_pts.empty()) { seed_p.dx = backward_pts[0].dx; seed_p.dy = backward_pts[0].dy; seed_p.dz = backward_pts[0].dz; }
        all_points.push_back(seed_p);

        all_points.insert(all_points.end(), forward_pts.begin(), forward_pts.end());

        // 위치 기반 정렬
        current_lane.points = all_points;
        if (current_lane.points.size() >= 2) {
            Lane current_lane;
            current_lane.id = lane_id_counter++;
            current_lane.points = all_points;
            current_lane.explicit_lane = true;
            current_lane.valid = true;

            LaneUtils::ReorderPoints(current_lane);

            lanes[current_lane.id] = current_lane;
        }
    }

    return lanes;
}

void RansacLaneGenerator::fitLocalLine(const std::vector<VoxelNode>& candidates, 
                                       std::vector<Point6D>& out_points, 
                                       std::vector<int>& out_inlier_indices) {
    if (candidates.size() < 2) return;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& node : candidates) {
        cloud->push_back(pcl::PointXYZ(node.point.x, node.point.y, node.point.z));
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(config_.distance_threshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < config_.min_inliers) return;

    out_inlier_indices = inliers->indices;

    // 라인 모델(점+방향)에서 시작점과 끝점 계산 (투영)
    // P = P0 + t * Dir
    Eigen::Vector3f p0(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    Eigen::Vector3f dir(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    dir.normalize();

    float seed_yaw = candidates[0].point.yaw;
    float line_yaw = std::atan2(dir.y(), dir.x()); // 벡터 -> yaw 변환

    // 시드점의 진행 방향과 반대라면 벡터 방향을 뒤집음
    if (std::abs(line_yaw - seed_yaw) > M_PI_2) { // 90도 이상 차이나면 반대 방향
        dir = -dir;
    }

    // 3. Inlier 점들을 직선상 거리(t) 기준으로 정렬하기 위한 벡터
    std::vector<std::pair<float, Point6D>> ordered_points;

    for (int idx : inliers->indices) {
        pcl::PointXYZ pt_pcl = cloud->points[idx];
        Eigen::Vector3f pt_vec(pt_pcl.x, pt_pcl.y, pt_pcl.z);
        
        // 직선 위의 점으로 투영(Projection)하여 거리 t 계산
        // P_proj = P0 + t * Dir -> t = (P - P0) dot Dir
        float t = (pt_vec - p0).dot(dir);
        
        Point6D p;
        // 원래 좌표를 그대로 쓸지, 직선에 투영된 좌표를 쓸지 결정 (보통 투영된 좌표가 깔끔함)
        // 여기서는 원래 좌표를 사용하되, 필요시 아래 주석 해제하여 투영 좌표 사용 가능
        // Eigen::Vector3f projected_pt = p0 + t * dir;
        // p.x = projected_pt.x(); p.y = projected_pt.y(); p.z = projected_pt.z();
        p.x = pt_pcl.x;
        p.y = pt_pcl.y;
        p.z = pt_pcl.z;

        // [핵심] 방향 정보 저장 (직선이므로 모든 점이 동일한 dir을 가짐)
        p.dx = dir.x();
        p.dy = dir.y();
        p.dz = dir.z();
        
        ordered_points.push_back({t, p});
    }

    // 4. 거리 t를 기준으로 오름차순 정렬 (Polyline 순서 보장)
    std::sort(ordered_points.begin(), ordered_points.end(), 
        [](const auto& a, const auto& b) { return a.first < b.first; });

    // 5. 결과 저장
    for (const auto& pair : ordered_points) {
        out_points.push_back(pair.second);
    }
}