#include "real_time_map/RansacLaneGenerator.h"
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

    // 1. 데이터 준비 (복사 및 인덱싱)
    std::vector<VoxelNode> nodes;
    nodes.reserve(voxels.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (int i = 0; i < voxels.size(); ++i) {
        nodes.push_back({i, voxels[i], false});
        cloud->push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // 2. 밀도 순으로 시드 정렬
    std::vector<int> sorted_indices(nodes.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return nodes[a].point.density > nodes[b].point.density;
    });

    int lane_id_counter = 0;
    float rad_threshold = config_.yaw_threshold * M_PI / 180.0f;

    // 3. Incremental RANSAC Loop
    for (int idx : sorted_indices) {
        if (nodes[idx].visited) continue;
        if (nodes[idx].point.density < config_.min_density) continue;

        // 새로운 차선 시작
        Lane current_lane;
        current_lane.id = lane_id_counter++;
        current_lane.type = 0; // Unknown
        current_lane.explicit_lane = true;
        current_lane.valid = true;

        // 시드 포인트 설정
        VoxelNode* current_node = &nodes[idx];
        
        // 양방향 확장 (일단은 단방향 전진만 구현, 필요시 loop로 양방향 가능)
        while (true) {
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            
            // 주변 점 탐색
            pcl::PointXYZ search_pt(current_node->point.x, current_node->point.y, current_node->point.z);
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) {
                break;
            }

            // 후보군 필터링 (방향성 체크 + 미방문)
            std::vector<VoxelNode> ransac_candidates;
            std::vector<int> candidate_global_indices;
            float seed_yaw = current_node->point.yaw;

            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue;

                // Yaw 차이 계산 (각도 정규화 고려)
                float diff = std::abs(nodes[ni].point.yaw - seed_yaw);
                // yaw 정규화가 깔끔하지 않을 수 있음
                while(diff > M_PI) diff -= 2 * M_PI;
                if (std::abs(diff) > rad_threshold) continue;

                ransac_candidates.push_back(nodes[ni]);
                candidate_global_indices.push_back(ni);
            }

            if (ransac_candidates.size() < config_.min_inliers) break;

            // 로컬 라인 피팅
            std::vector<Point6D> fitted_points;
            std::vector<int> inlier_local_indices;
            
            fitLocalLine(ransac_candidates, fitted_points, inlier_local_indices);

            if (fitted_points.size() < 2) break; // 피팅 실패

            // 결과 저장 및 상태 업데이트
            for (const auto& pt : fitted_points) {
                current_lane.points.push_back(pt);
            }

            // Inlier들을 방문 처리
            for (int local_idx : inlier_local_indices) {
                int global_idx = candidate_global_indices[local_idx];
                nodes[global_idx].visited = true;
            }

            // 다음 확장을 위한 시드 업데이트 (세그먼트의 끝점과 가장 가까운 방문 안 된 점 찾기..는 복잡하므로)
            // 간단하게: 세그먼트의 끝점 위치를 기준으로 다시 검색
            // 실제 구현에서는 fitted_points.back()에 해당하는 VoxelNode를 찾아야 함.
            // 여기서는 근사적으로 가장 마지막 Inlier를 다음 시드로 잡음
            int last_inlier_global_idx = candidate_global_indices[inlier_local_indices.back()];
            current_node = &nodes[last_inlier_global_idx];
            
            // 만약 더이상 확장할 곳이 없으면 break (이 부분은 정교한 로직 필요)
            // 여기서는 단순 루프로 진행
        }

        if (current_lane.points.size() >= 2) {
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

    // inlier 점들을 직선에 투영하여 t값(위치) 계산
    float min_t = std::numeric_limits<float>::max();
    float max_t = std::numeric_limits<float>::lowest();

    for (int idx : inliers->indices) {
        Eigen::Vector3f pt(cloud->points[idx].x, cloud->points[idx].y, cloud->points[idx].z);
        float t = (pt - p0).dot(dir);
        if (t < min_t) min_t = t;
        if (t > max_t) max_t = t;
    }

    // 시작점과 끝점 생성
    Eigen::Vector3f start_pt = p0 + min_t * dir;
    Eigen::Vector3f end_pt = p0 + max_t * dir;

    // 시드점의 진행 방향과 매칭 (역방향 방지)
    // candidates[0]는 보통 시드점과 가까움
    float seed_yaw = candidates[0].point.yaw;
    float line_yaw = std::atan2(dir.y(), dir.x());
    
    // 내적을 통해 방향 확인 (방향이 반대면 스왑)
    if (std::abs(line_yaw - seed_yaw) > M_PI_2) {
        std::swap(start_pt, end_pt);
        dir = -dir;
    }

    Point6D p1, p2;
    p1.x = start_pt.x(); p1.y = start_pt.y(); p1.z = start_pt.z();
    p2.x = end_pt.x(); p2.y = end_pt.y(); p2.z = end_pt.z();
    
    // 단순화: 시작점과 끝점만 저장 (나중에 spline 등으로 보간 가능)
    out_points.push_back(p1);
    out_points.push_back(p2);
}