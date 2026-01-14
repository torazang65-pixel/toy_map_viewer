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
        std::vector<Point6D> final_projected_points;
        
        // 1. 상태 변수 초기화
        std::vector<int> current_inliers; 
        current_inliers.push_back(start_node_idx); // 초기 시드 포함
        
        VoxelNode* search_center_node = &nodes[start_node_idx];
        
        // 초기 직선 모델 (2D 평면 z=0 가정)
        // 시드점의 Yaw를 이용해 초기 방향 벡터 설정
        Eigen::Vector3f curr_p0(search_center_node->point.x, search_center_node->point.y, 0.0f);
        float yaw = search_center_node->point.yaw;
        if (!is_forward) yaw += M_PI; 
        Eigen::Vector3f curr_dir(std::cos(yaw), std::sin(yaw), 0.0f);

        bool model_initialized = false;

        while (true) {
            // ---------------------------------------------------------
            // 2. 주변 점 탐색 (Radius Search - 3D 거리 기준)
            // ---------------------------------------------------------
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            // 검색은 3D 좌표로 수행 (도로의 높낮이 반영)
            pcl::PointXYZ search_pt(search_center_node->point.x, search_center_node->point.y, search_center_node->point.z);
            
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) break;

            std::vector<int> new_candidates;
            
            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue;
                // 중복 방지
                if (std::find(current_inliers.begin(), current_inliers.end(), ni) != current_inliers.end()) continue;

                // [추가] Z축 허용 범위 필터링 (검색 중심점 기준)
                if (std::abs(nodes[ni].point.z - search_center_node->point.z) > config_.z_tolerance) continue;

                // 거리/방향 계산을 위해 2D 투영 (z=0)
                Eigen::Vector3f pt_2d(nodes[ni].point.x, nodes[ni].point.y, 0.0f);

                // 1) 방향 체크 (3D Yaw 사용)
                float pt_yaw = nodes[ni].point.yaw;
                float model_yaw = std::atan2(curr_dir.y(), curr_dir.x());
                float yaw_diff = std::abs(pt_yaw - model_yaw);
                while(yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                if (std::abs(yaw_diff) > rad_threshold) continue;

                // 2) 거리 체크 (2D 평면상 직선과의 거리)
                if (model_initialized) {
                    float dist_to_line = (pt_2d - curr_p0).cross(curr_dir).norm();
                    if (dist_to_line > 0.5f) continue; // 50cm 이내
                    
                    // 진행 방향(앞쪽)에 있는 점인지 체크
                    if ((pt_2d - curr_p0).dot(curr_dir) < -0.5f) continue; // 약간의 후방은 허용(-0.5)
                }

                new_candidates.push_back(ni);
            }

            if (new_candidates.empty()) break;

            // ---------------------------------------------------------
            // 3. 모델 재피팅 (Global Line Refinement)
            // ---------------------------------------------------------
            // 현재까지의 모든 점(기존 inliers + 새 후보)을 합쳐서 RANSAC을 다시 돌림
            std::vector<int> temp_inliers = current_inliers;
            temp_inliers.insert(temp_inliers.end(), new_candidates.begin(), new_candidates.end());

            if (temp_inliers.size() < config_.min_inliers) break;

            // [핵심] RANSAC용 클라우드 생성 시 Z=0으로 강제 투영 (2D Line Fitting 유도)
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_2d(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : temp_inliers) {
                temp_cloud_2d->push_back(pcl::PointXYZ(nodes[idx].point.x, nodes[idx].point.y, 0.0f));
            }

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr sac_inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config_.distance_threshold); // 2D 거리 허용 오차
            seg.setInputCloud(temp_cloud_2d);
            seg.segment(*sac_inliers, *coefficients);

            if (sac_inliers->indices.empty()) break;

            // ---------------------------------------------------------
            // 4. 모델 업데이트
            // ---------------------------------------------------------
            model_initialized = true;
            
            // 새 모델 파라미터 (Z=0인 직선)
            Eigen::Vector3f new_p0(coefficients->values[0], coefficients->values[1], 0.0f);
            Eigen::Vector3f new_dir(coefficients->values[3], coefficients->values[4], 0.0f);
            new_dir.normalize();

            // 방향 정렬 (이전 진행 방향과 반대면 뒤집기)
            if (curr_dir.dot(new_dir) < 0) {
                new_dir = -new_dir;
            }
            
            // 모델 갱신
            curr_p0 = new_p0;
            curr_dir = new_dir;

            // ---------------------------------------------------------
            // 5. Inlier 리스트 업데이트
            // ---------------------------------------------------------
            // RANSAC 결과로 선택된 인덱스들만 실제 Inlier로 확정
            std::vector<int> refined_inliers;
            for (int idx : sac_inliers->indices) {
                refined_inliers.push_back(temp_inliers[idx]);
            }
            
            // 더 이상 확장이 안 되면(새로운 점이 추가되지 않았거나 줄어들면) 종료
            if (refined_inliers.size() <= current_inliers.size()) break;

            current_inliers = refined_inliers;

            // ---------------------------------------------------------
            // 6. 다음 탐색 중심 설정 (가장 끝 점 찾기)
            // ---------------------------------------------------------
            float max_t = -std::numeric_limits<float>::max();
            int best_end_idx = -1;

            for (int idx : current_inliers) {
                // 투영 거리 t 계산 (2D 상에서 수행)
                Eigen::Vector3f pt_2d(nodes[idx].point.x, nodes[idx].point.y, 0.0f);
                float t = (pt_2d - curr_p0).dot(curr_dir);
                if (t > max_t) {
                    max_t = t;
                    best_end_idx = idx;
                }
            }
            
            if (best_end_idx != -1) {
                // 다음 검색은 해당 점의 *실제 3D 좌표*를 중심으로 수행
                search_center_node = &nodes[best_end_idx];
            } else {
                break;
            }
        }

        // ---------------------------------------------------------
        // 7. 최종 투영 및 결과 반환
        // ---------------------------------------------------------
        if (current_inliers.size() < 2) return {};

        for(int idx : current_inliers) nodes[idx].visited = true;

        std::vector<std::pair<float, Point6D>> sorted_pts;
        for (int idx : current_inliers) {
            // 정렬 기준 t는 2D 직선 상의 투영 거리
            Eigen::Vector3f pt_2d(nodes[idx].point.x, nodes[idx].point.y, 0.0f);
            float t = (pt_2d - curr_p0).dot(curr_dir);
            
            // 저장은 *원본 3D 좌표*를 사용
            Point6D p;
            p.x = nodes[idx].point.x;
            p.y = nodes[idx].point.y;
            p.z = nodes[idx].point.z; 
            
            // 방향 벡터는 2D로 구한 것 (z=0)
            p.dx = curr_dir.x(); p.dy = curr_dir.y(); p.dz = 0.0;
            
            sorted_pts.push_back({t, p});
        }
        
        std::sort(sorted_pts.begin(), sorted_pts.end(), 
            [](const auto& a, const auto& b){ return a.first < b.first; });

        for(auto& pair : sorted_pts) final_projected_points.push_back(pair.second);

        return final_projected_points;
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

        all_points.insert(all_points.end(), forward_pts.begin(), forward_pts.end());

        // 위치 기반 정렬
        if (all_points.size() >= 2) {
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
        
        Eigen::Vector3f projected_pt = p0 + t * dir;

        Point6D p;
        p.x = projected_pt.x();
        p.y = projected_pt.y();
        p.z = projected_pt.z();

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