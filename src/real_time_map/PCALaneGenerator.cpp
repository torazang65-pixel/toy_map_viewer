#include "real_time_map/PCALaneGenerator.h"
#include "toy_map_viewer/LaneUtils.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <cmath>
#include <iostream>

PCALaneGenerator::PCALaneGenerator(const Config& config) : config_(config) {}

std::map<int, Lane> PCALaneGenerator::generate(const std::vector<VoxelPoint>& voxels) {
    std::map<int, Lane> lanes;
    if (voxels.empty()) return lanes;

    // Prepare KD-tree.
    std::vector<VoxelNode> nodes;
    nodes.reserve(voxels.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < voxels.size(); ++i) {
        nodes.push_back({i, voxels[i], false});
        cloud->push_back(pcl::PointXYZ(voxels[i].x, voxels[i].y, voxels[i].z));
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // Seed order: density desc.
    std::vector<int> sorted_indices(nodes.size());
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(), [&](int a, int b) {
        return nodes[a].point.density > nodes[b].point.density;
    });

    int lane_id_counter = 0;
    float rad_threshold = config_.yaw_threshold * M_PI / 180.0f;

    // Expand one lane from a seed.
    auto expand_lane = [&](int start_node_idx, bool is_forward) -> std::vector<Point6D> {
        std::vector<Point6D> final_projected_points;

        std::vector<int> current_inliers;
        current_inliers.push_back(start_node_idx);

        std::vector<int> newly_visited_points;

        VoxelNode* search_center_node = &nodes[start_node_idx];

        Eigen::Vector3f curr_p0(search_center_node->point.x, search_center_node->point.y, 0.0f);
        Eigen::Vector3f start_point = curr_p0;
        float yaw = search_center_node->point.yaw;
        if (!is_forward) yaw += M_PI;
        Eigen::Vector3f curr_dir(std::cos(yaw), std::sin(yaw), 0.0f);

        bool model_initialized = false;

        while (true) {
            // Radius search in 3D, fit in 2D.
            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_dists;
            pcl::PointXYZ search_pt(search_center_node->point.x, search_center_node->point.y, search_center_node->point.z);
            
            if (kdtree.radiusSearch(search_pt, config_.search_radius, neighbor_indices, neighbor_dists) <= 0) break;

            std::vector<int> new_candidates;
            
            for (int ni : neighbor_indices) {
                if (nodes[ni].visited) continue;
                
                bool already_in = false;
                for(int exist : current_inliers) if(exist == ni) { already_in = true; break; }
                if(already_in) continue;

                if (std::abs(nodes[ni].point.z - search_center_node->point.z) > config_.z_tolerance) continue;
                Eigen::Vector3f pt_2d(nodes[ni].point.x, nodes[ni].point.y, 0.0f);

                float pt_yaw = nodes[ni].point.yaw;
                float model_yaw = std::atan2(curr_dir.y(), curr_dir.x());
                float yaw_diff = std::abs(pt_yaw - model_yaw);
                while(yaw_diff > M_PI) yaw_diff -= 2 * M_PI;
                if (std::abs(yaw_diff) > rad_threshold) continue;

                float dist_to_line = (pt_2d - curr_p0).cross(curr_dir).norm();
                if (dist_to_line > config_.distance_threshold) continue;

                if (model_initialized) {
                    if ((pt_2d - curr_p0).dot(curr_dir) <= 0) continue; 
                }

                new_candidates.push_back(ni);
            }

            if (new_candidates.empty()) break;

            std::vector<int> temp_inliers = current_inliers;
            temp_inliers.insert(temp_inliers.end(), new_candidates.begin(), new_candidates.end());

            if (temp_inliers.size() < config_.min_inliers) break;

            Eigen::Vector2f centroid(0.0f, 0.0f);
            for (int idx : temp_inliers) {
                centroid.x() += nodes[idx].point.x;
                centroid.y() += nodes[idx].point.y;
            }
            centroid /= static_cast<float>(temp_inliers.size());

            float cov_xx = 0.0f, cov_xy = 0.0f, cov_yy = 0.0f;
            for (int idx : temp_inliers) {
                float dx = nodes[idx].point.x - centroid.x();
                float dy = nodes[idx].point.y - centroid.y();
                cov_xx += dx * dx;
                cov_xy += dx * dy;
                cov_yy += dy * dy;
            }

            // Principal direction in 2D.
            float trace = cov_xx + cov_yy;
            float det = cov_xx * cov_yy - cov_xy * cov_xy;
            float lambda1 = trace / 2.0f + std::sqrt(trace * trace / 4.0f - det);

            Eigen::Vector2f eigen_vec;
            if (std::abs(cov_xy) > 1e-6) {
                eigen_vec.x() = lambda1 - cov_yy;
                eigen_vec.y() = cov_xy;
                eigen_vec.normalize();
            } else {
                eigen_vec = (cov_xx > cov_yy) ? Eigen::Vector2f(1.0f, 0.0f) : Eigen::Vector2f(0.0f, 1.0f);
            }

            // Validate PCA direction with average yaw.
            float mean_sin_2yaw = 0.0f;
            float mean_cos_2yaw = 0.0f;
            for (int idx : temp_inliers) {
                float double_yaw = 2.0f * nodes[idx].point.yaw;
                mean_sin_2yaw += std::sin(double_yaw);
                mean_cos_2yaw += std::cos(double_yaw);
            }
            float avg_double_yaw = std::atan2(mean_sin_2yaw, mean_cos_2yaw);
            float avg_yaw = avg_double_yaw / 2.0f;

            Eigen::Vector2f yaw_dir(std::cos(avg_yaw), std::sin(avg_yaw));

            float alignment = std::abs(eigen_vec.dot(yaw_dir));

            if (alignment < 0.867f) {
                for (int idx : newly_visited_points) {
                    nodes[idx].visited = false;
                }
                return {};
            }

            model_initialized = true;

            Eigen::Vector3f new_p0(centroid.x(), centroid.y(), 0.0f);
            Eigen::Vector3f new_dir(eigen_vec.x(), eigen_vec.y(), 0.0f);

            float seed_yaw = nodes[start_node_idx].point.yaw;
            if(!is_forward) seed_yaw += M_PI;
            Eigen::Vector3f seed_dir(std::cos(seed_yaw), std::sin(seed_yaw), 0.0f);

            if (new_dir.dot(seed_dir) < 0) {
                new_dir = -new_dir;
            }

            curr_p0 = new_p0;
            curr_dir = new_dir;

            if (temp_inliers.size() <= current_inliers.size()) break;

            current_inliers = temp_inliers;

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
                Eigen::Vector3f end_point(nodes[best_end_idx].point.x, nodes[best_end_idx].point.y, 0.0f);
                float current_length = (end_point - start_point).norm();

                if (current_length > config_.max_lane_length) {
                    break;
                }

                search_center_node = &nodes[best_end_idx];
            } else {
                break;
            }
        }

        if (current_inliers.size() < 2) return {};

        for(int idx : current_inliers) {
            if (!nodes[idx].visited) {
                newly_visited_points.push_back(idx);
            }
            nodes[idx].visited = true;
        }

        std::vector<std::pair<float, Point6D>> sorted_pts;
        for (int idx : current_inliers) {
            Eigen::Vector3f pt_2d(nodes[idx].point.x, nodes[idx].point.y, 0.0f);
            float t = (pt_2d - curr_p0).dot(curr_dir);

            Eigen::Vector3f proj_2d = curr_p0 + t * curr_dir;

            Point6D p;
            p.x = proj_2d.x();
            p.y = proj_2d.y();
            p.z = nodes[idx].point.z;

            // 원래 voxel 방향과 직선 방향을 blend
            float original_yaw = nodes[idx].point.yaw;
            Eigen::Vector3f original_dir(std::cos(original_yaw), std::sin(original_yaw), 0.0f);

            // 원래 방향이 직선 방향과 얼마나 일치하는지 계산
            float alignment = original_dir.dot(curr_dir);

            // 방향이 반대면 뒤집기
            Eigen::Vector3f aligned_original_dir = original_dir;
            if (alignment < 0) {
                aligned_original_dir = -original_dir;
                alignment = -alignment;
            }

            // alignment가 높을수록 직선 방향 가중치를 높임 (0.7~0.95 범위로 매핑)
            // alignment: 1.0 (완전 일치) -> weight = 0.95 (거의 직선)
            // alignment: 0.0 (수직)     -> weight = 0.7  (원본 방향 더 보존)
            float line_weight = 0.7f + 0.25f * alignment;

            // 가중 평균으로 방향 보정
            Eigen::Vector3f blended_dir = line_weight * curr_dir + (1.0f - line_weight) * aligned_original_dir;
            blended_dir.normalize();

            p.dx = blended_dir.x();
            p.dy = blended_dir.y();
            p.dz = 0.0;

            sorted_pts.push_back({t, p});
        }
        
        std::sort(sorted_pts.begin(), sorted_pts.end(), 
            [](const auto& a, const auto& b){ return a.first < b.first; });

        for(auto& pair : sorted_pts) final_projected_points.push_back(pair.second);

        return final_projected_points;
    };

    for (int idx : sorted_indices) {
        if (nodes[idx].visited) continue;
        if (nodes[idx].point.density < config_.min_density) continue;

        nodes[idx].visited = true;

        std::vector<Point6D> forward_pts = expand_lane(idx, true);

        std::vector<Point6D> backward_pts = expand_lane(idx, false);

        std::reverse(backward_pts.begin(), backward_pts.end());

        std::vector<Point6D> all_points = backward_pts;
        all_points.insert(all_points.end(), forward_pts.begin(), forward_pts.end());

        if (all_points.size() >= 2) {
            std::vector<Point6D> deduplicated;
            deduplicated.push_back(all_points[0]);

            for (size_t i = 1; i < all_points.size(); ++i) {
                const auto& prev = deduplicated.back();
                const auto& curr = all_points[i];
                float dx = curr.x - prev.x;
                float dy = curr.y - prev.y;
                float dist = std::sqrt(dx*dx + dy*dy);

                if (dist >= 0.1f) {
                    deduplicated.push_back(curr);
                }
            }
            all_points = deduplicated;
        }

        if (all_points.size() >= 2) {
            Lane current_lane;
            current_lane.id = lane_id_counter++;
            current_lane.points = all_points;
            current_lane.explicit_lane = true;
            current_lane.valid = true;

            LaneUtils::ReorderPoints(current_lane);

            lanes[current_lane.id] = current_lane;

            // Sweep nearby points to avoid duplicates.
            for (const auto& lane_pt : current_lane.points) {
                std::vector<int> sweep_indices;
                std::vector<float> sweep_dists;
                pcl::PointXYZ sweep_center(lane_pt.x, lane_pt.y, lane_pt.z);

                if (kdtree.radiusSearch(sweep_center, config_.search_radius, sweep_indices, sweep_dists) > 0) {
                    for (int sweep_idx : sweep_indices) {
                        if (!nodes[sweep_idx].visited) {
                            float lane_yaw = std::atan2(lane_pt.dy, lane_pt.dx);
                            float pt_yaw = nodes[sweep_idx].point.yaw;
                            float yaw_diff = std::abs(lane_yaw - pt_yaw);

                            if (std::abs(yaw_diff) < 1.5 * rad_threshold) {
                                nodes[sweep_idx].visited = true;
                            }
                        }
                    }
                }
            }
        }
    }

    return lanes;
}

void PCALaneGenerator::fitLocalLine(const std::vector<VoxelNode>& candidates, 
                                       std::vector<Point6D>& out_points, 
                                       std::vector<int>& out_inlier_indices) {
}
