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
        std::vector<int> current_inliers; // 현재 세그먼트에 포함된 모든 점들의 Global Index
        current_inliers.push_back(start_node_idx); // 시드점 포함
        
        VoxelNode* search_center_node = &nodes[start_node_idx];
        
        // 초기 직선 모델 (시드점의 Yaw로 임시 초기화)
        Eigen::Vector3f curr_p0(search_center_node->point.x, search_center_node->point.y, search_center_node->point.z);
        float yaw = search_center_node->point.yaw;
        if (!is_forward) yaw += M_PI; // 후방이면 반대 방향
        Eigen::Vector3f curr_dir(std::cos(yaw), std::sin(yaw), 0.0f);

        bool model_initialized = false;

        while (true) {
            // 2. 주변 점 탐색 (Radius Search)
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            pcl::PointXYZ search_pt(search_center_node->point.x, search_center_node->point.y, search_center_node->point.z);
            
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) break;

            std::vector<int> new_candidates;
            
            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue; // 이미 방문한 점 제외
                // 이미 현재 리스트에 있는 점 제외 (중복 방지)
                if (std::find(current_inliers.begin(), current_inliers.end(), ni) != current_inliers.end()) continue;

                Eigen::Vector3f pt(nodes[ni].point.x, nodes[ni].point.y, nodes[ni].point.z);

                // [핵심] 기존 직선 모델을 이용한 필터링
                // 1) 방향 체크 (기존 로직 유지 - 역주행 방지)
                float pt_yaw = nodes[ni].point.yaw;
                float model_yaw = std::atan2(curr_dir.y(), curr_dir.x());
                float yaw_diff = std::abs(pt_yaw - model_yaw);
                while(yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                if (std::abs(yaw_diff) > rad_threshold) continue;

                // 2) 거리 체크 (점과 직선 사이의 거리)
                // 만약 모델이 초기화되었다면, 직선과의 거리를 봅니다.
                // 초기화 전이라면(첫 루프) 그냥 탐색 반경 내면 ok.
                if (model_initialized) {
                    // 점과 직선 사이의 거리: || (P - P0) cross Dir ||
                    float dist_to_line = (pt - curr_p0).cross(curr_dir).norm();
                    if (dist_to_line > 0.5f) continue; // 직선에서 50cm 이상 벗어나면 탈락 (엄격하게)
                    
                    // 진행 방향 체크 (내적 > 0)
                    if ((pt - curr_p0).dot(curr_dir) < 0) continue; // 뒤에 있는 점 제외
                }

                new_candidates.push_back(ni);
            }

            if (new_candidates.empty()) break;

            // 3. 후보군을 누적하고 전체에 대해 재피팅 (Refit)
            // 임시로 합쳐서 RANSAC 돌려봄
            std::vector<int> temp_inliers = current_inliers;
            temp_inliers.insert(temp_inliers.end(), new_candidates.begin(), new_candidates.end());

            if (temp_inliers.size() < config_.min_inliers) break;

            // RANSAC 수행 (전체 점 대상)
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int idx : temp_inliers) {
                temp_cloud->push_back(pcl::PointXYZ(nodes[idx].point.x, nodes[idx].point.y, nodes[idx].point.z));
            }

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr sac_inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_LINE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(config_.distance_threshold);
            seg.setInputCloud(temp_cloud);
            seg.segment(*sac_inliers, *coefficients);

            if (sac_inliers->indices.empty()) break;

            // 4. 모델 업데이트 및 확정
            model_initialized = true;
            
            // 새 모델 파라미터 추출
            curr_p0 = Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            curr_dir = Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
            curr_dir.normalize();

            // 방향 정렬 (이전 방향과 내적하여 뒤집힘 방지)
            // (첫 루프에서는 시드 방향 따르고, 그 뒤론 관성 유지)
            float check_yaw = (current_inliers.size() == 1) ? 
                              (is_forward ? nodes[start_node_idx].point.yaw : nodes[start_node_idx].point.yaw + M_PI) :
                              std::atan2(curr_dir.y(), curr_dir.x());
            
            // 모델의 dir이 우리가 원하는 진행 방향과 반대면 뒤집음
            // (RANSAC은 선의 방향성(+-180)을 모르므로)
            // 여기서는 간단히: 새로 구한 dir이 기존 확장의 방향(End - Start)과 같은지 체크
            // 하지만 누적 피팅이므로, dir 자체의 부호만 맞춰주면 됨.
            
            // 5. Inlier 업데이트 (RANSAC이 걸러낸 진짜 Inlier만 남김)
            // temp_inliers 중에서 sac_inliers 인덱스에 해당하는 것들만 추출
            std::vector<int> refined_inliers;
            for (int idx : sac_inliers->indices) {
                refined_inliers.push_back(temp_inliers[idx]);
            }
            
            // 만약 확장이 멈췄거나(새로운 점이 하나도 Inlier가 안 됨) 하면 종료
            if (refined_inliers.size() <= current_inliers.size()) break;

            current_inliers = refined_inliers;

            // 6. 다음 탐색을 위해 '가장 끝 점' 찾기
            // 현재 모델(직선) 상에서 가장 멀리 있는 점(t가 최대인 점)을 찾음
            float max_t = -std::numeric_limits<float>::max();
            int best_end_idx = -1;

            for (int idx : current_inliers) {
                Eigen::Vector3f pt(nodes[idx].point.x, nodes[idx].point.y, nodes[idx].point.z);
                float t = (pt - curr_p0).dot(curr_dir);
                if (t > max_t) {
                    max_t = t;
                    best_end_idx = idx;
                }
            }
            
            if (best_end_idx != -1) {
                search_center_node = &nodes[best_end_idx];
            } else {
                break;
            }
        }

        // 7. 최종 투영 및 저장 (Loop 종료 후 한 번만 수행)
        if (current_inliers.size() < 2) return {};

        // Inlier 점들을 방문 처리
        for(int idx : current_inliers) nodes[idx].visited = true;

        // 투영 및 정렬
        std::vector<std::pair<float, Point6D>> sorted_pts;
        for (int idx : current_inliers) {
            Eigen::Vector3f pt(nodes[idx].point.x, nodes[idx].point.y, nodes[idx].point.z);
            float t = (pt - curr_p0).dot(curr_dir);
            Eigen::Vector3f proj = curr_p0 + t * curr_dir;
            
            Point6D p;
            p.x = proj.x(); p.y = proj.y(); p.z = proj.z();
            p.dx = curr_dir.x(); p.dy = curr_dir.y(); p.dz = curr_dir.z();
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