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

    // 2. 시드 정렬 (밀도 순)
    std::vector<int> sorted_indices(nodes.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return nodes[a].point.density > nodes[b].point.density;
    });

    int lane_id_counter = 0;
    float rad_threshold = config_.yaw_threshold * M_PI / 180.0f;

    // ---------------------------------------------------------
    // Lambda: Lane Expansion
    // ---------------------------------------------------------
    auto expand_lane = [&](int start_node_idx, bool is_forward) -> std::vector<Point6D> {
        std::vector<Point6D> final_projected_points;
        
        // 초기화
        std::vector<int> current_inliers; 
        current_inliers.push_back(start_node_idx);
        
        VoxelNode* search_center_node = &nodes[start_node_idx];
        
        // 초기 모델: 2D 평면(z=0) 투영
        Eigen::Vector3f curr_p0(search_center_node->point.x, search_center_node->point.y, 0.0f);
        float yaw = search_center_node->point.yaw;
        if (!is_forward) yaw += M_PI; 
        Eigen::Vector3f curr_dir(std::cos(yaw), std::sin(yaw), 0.0f);

        bool model_initialized = false;

        while (true) {
            // 2.1 주변 점 탐색 (Radius Search - 3D 거리 기준)
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            pcl::PointXYZ search_pt(search_center_node->point.x, search_center_node->point.y, search_center_node->point.z);
            
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) break;

            std::vector<int> new_candidates;
            
            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue;
                
                // 이미 포함된 점 제외
                bool already_in = false;
                for(int exist : current_inliers) if(exist == ni) { already_in = true; break; }
                if(already_in) continue;

                // [Z축 필터링] 검색 중심점과 높이 차이가 허용 범위 이내여야 함
                if (std::abs(nodes[ni].point.z - search_center_node->point.z) > config_.z_tolerance) continue;

                // [2D 투영] 거리/방향 계산을 위해 z=0으로 설정
                Eigen::Vector3f pt_2d(nodes[ni].point.x, nodes[ni].point.y, 0.0f);

                // 1) 방향 체크 (3D yaw 사용)
                float pt_yaw = nodes[ni].point.yaw;
                float model_yaw = std::atan2(curr_dir.y(), curr_dir.x());
                float yaw_diff = std::abs(pt_yaw - model_yaw);
                while(yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                if (std::abs(yaw_diff) > rad_threshold) continue;

                // 2) 거리 체크 (2D 평면 상의 직선과의 거리)
                if (model_initialized) {
                    float dist_to_line = (pt_2d - curr_p0).cross(curr_dir).norm();
                    if (dist_to_line > config_.distance_threshold) continue;
                    
                    // 진행 방향 체크 (후방 -0.5m 까지는 허용)
                    if ((pt_2d - curr_p0).dot(curr_dir) < -0.5f) continue; 
                }

                new_candidates.push_back(ni);
            }

            if (new_candidates.empty()) break;

            // 2.2 모델 재피팅 (Global Line Refinement)
            // 현재까지의 모든 점(기존 + 신규)을 합쳐서 RANSAC 수행
            std::vector<int> temp_inliers = current_inliers;
            temp_inliers.insert(temp_inliers.end(), new_candidates.begin(), new_candidates.end());

            if (temp_inliers.size() < config_.min_inliers) break;

            // [핵심] RANSAC용 클라우드: Z=0으로 강제하여 2D Line Fitting 유도
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
            seg.setDistanceThreshold(config_.distance_threshold); // 2D 거리 임계값 사용
            seg.setInputCloud(temp_cloud_2d);
            seg.segment(*sac_inliers, *coefficients);

            if (sac_inliers->indices.empty()) break;

            // 2.3 모델 업데이트
            model_initialized = true;
            
            // 새 모델 파라미터 (Z=0인 직선)
            Eigen::Vector3f new_p0(coefficients->values[0], coefficients->values[1], 0.0f);
            Eigen::Vector3f new_dir(coefficients->values[3], coefficients->values[4], 0.0f);
            new_dir.normalize();

            // 초기 시드 방향과 반대라면 뒤집기 (일관성 유지)
            float seed_yaw = nodes[start_node_idx].point.yaw;
            if(!is_forward) seed_yaw += M_PI;
            Eigen::Vector3f seed_dir(std::cos(seed_yaw), std::sin(seed_yaw), 0.0f);

            if (new_dir.dot(seed_dir) < 0) {
                new_dir = -new_dir;
            }
            
            curr_p0 = new_p0;
            curr_dir = new_dir;

            // 2.4 Inlier 리스트 업데이트 (RANSAC 결과만 남김)
            std::vector<int> refined_inliers;
            for (int idx : sac_inliers->indices) {
                refined_inliers.push_back(temp_inliers[idx]);
            }
            
            // 확장이 멈췄거나(새 점이 inlier가 안 됨) 줄어들면 종료
            if (refined_inliers.size() <= current_inliers.size()) break;

            current_inliers = refined_inliers;

            // 2.5 다음 탐색 중심 설정 (2D 직선 상에서 가장 멀리 있는 점)
            float max_t = -std::numeric_limits<float>::max();
            int best_end_idx = -1;

            for (int idx : current_inliers) {
                Eigen::Vector3f pt_2d(nodes[idx].point.x, nodes[idx].point.y, 0.0f);
                float t = (pt_2d - curr_p0).dot(curr_dir);
                if (t > max_t) {
                    max_t = t;
                    best_end_idx = idx;
                }
            }
            
            if (best_end_idx != -1) {
                // 다음 검색은 해당 점의 *실제 3D 좌표*를 중심으로 수행하여 로컬 높이를 반영
                search_center_node = &nodes[best_end_idx];
            } else {
                break;
            }
        }

        // 3. 최종 투영 및 저장
        if (current_inliers.size() < 2) return {};

        // Inlier 방문 처리
        for(int idx : current_inliers) nodes[idx].visited = true;

        std::vector<std::pair<float, Point6D>> sorted_pts;
        for (int idx : current_inliers) {
            // 정렬 기준 t는 2D 직선 상의 투영 거리
            Eigen::Vector3f pt_2d(nodes[idx].point.x, nodes[idx].point.y, 0.0f);
            float t = (pt_2d - curr_p0).dot(curr_dir);
            
            // 점 투영: P_proj = P0 + t * Dir (XY 평면상 직선)
            Eigen::Vector3f proj_2d = curr_p0 + t * curr_dir;

            // *중요* 저장은 투영된 XY좌표 + 원본 Z좌표를 사용
            Point6D p;
            p.x = proj_2d.x();
            p.y = proj_2d.y();
            p.z = nodes[idx].point.z; // 원본 높이 유지
            
            // 방향 벡터는 2D로 구한 것 저장 (z=0)
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

        // 양방향 확장
        std::vector<Point6D> forward_pts = expand_lane(idx, true);
        std::vector<Point6D> backward_pts = expand_lane(idx, false);
        
        // 합치기 (Back 뒤집을 필요 없음, 이미 방향 고려됨) -> 아님, 뒤집어야 함?
        // expand_lane 로직 상 backward는 역방향으로 뻗어나가므로, t값이 증가하는 순서로 정렬되어 있음.
        // 즉, [Seed -> ... -> End] 순서임.
        // 전체 Lane을 만들려면 [BackEnd -> ... -> Seed -> ... -> FwdEnd] 순이어야 함.
        // 따라서 Backward 결과는 뒤집어야 함.
        std::reverse(backward_pts.begin(), backward_pts.end());

        std::vector<Point6D> all_points = backward_pts;
        all_points.insert(all_points.end(), forward_pts.begin(), forward_pts.end());

        // 중복 제거 (Seed 부근) - 간단히 거리 기반으로 너무 가까운 점 제거
        if (all_points.size() >= 2) {
            Lane current_lane;
            current_lane.id = lane_id_counter++;
            current_lane.points = all_points;
            current_lane.explicit_lane = true;
            current_lane.valid = true;

            LaneUtils::ReorderPoints(current_lane); // 최종 정리

            lanes[current_lane.id] = current_lane;

            // =========================================================
            // [Sweeping Logic] 주변 노이즈 제거 (Visited 처리)
            // =========================================================
            for (const auto& pt : current_lane.points) {
                std::vector<int> sweep_indices;
                std::vector<float> sweep_dists;
                pcl::PointXYZ p(pt.x, pt.y, pt.z);
                
                // 생성된 차선 주변 config_.sweep_radius 내의 모든 점을 visited 처리
                if (kdtree.radiusSearch(p, config_.sweep_radius, sweep_indices, sweep_dists) > 0) {
                    for (int n_idx : sweep_indices) {
                        nodes[n_idx].visited = true;
                    }
                }
            }
        }
    }

    return lanes;
}

// fitLocalLine 함수는 이제 expand_lane 내부 로직으로 대체되었으므로 사용되지 않거나
// 필요 시 다른 용도로 유지할 수 있습니다.
void RansacLaneGenerator::fitLocalLine(const std::vector<VoxelNode>& candidates, 
                                       std::vector<Point6D>& out_points, 
                                       std::vector<int>& out_inlier_indices) {
    // (구현 생략 - 위 로직에 통합됨)
}