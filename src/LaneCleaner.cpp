#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <algorithm> // std::max, std::min
#include "toy_map_viewer/DataTypes.h"
#include "toy_map_viewer/LaneUtils.h" // CalculateLaneLength 사용

namespace LaneCleaner {
    struct PointSourceInfo {
        int lane_id;
        int index_in_lane;
    };
    struct LaneMetric {
        int id;
        double length;
        double linearity;
    };

    void TrimOverlappingLaneEnds(std::map<int, Lane>& global_map, const LaneConfig& config){
        const double OVERLAP_RADIUS = config.overlap_radius; // 30cm 이내
        const double LINEARITY_TOLERANCE = 1e-3;

        std::vector<LaneMetric> lane_metrics;
        lane_metrics.reserve(global_map.size());

        int total_points = 0;
        
        // 1. Calculate lane metrics
        for (const auto& pair : global_map) {
            double length = LaneUtils::CalculateLaneLength(pair.second);
            double linearity = LaneUtils::CalculateLaneLinearity(pair.second);
            lane_metrics.push_back({pair.first, length, linearity});
            total_points += pair.second.points.size();
        }

        // 2. Build point cloud for KD-Tree
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<PointSourceInfo> cloud_meta;
        std::map<int, int> lane_start_indices;

        cloud->reserve(total_points);
        cloud_meta.reserve(total_points);

        int current_global_index = 0;
        for (const auto& pair : global_map) {
            int lane_id = pair.first;
            const Lane& lane = pair.second;

            lane_start_indices[lane_id] = current_global_index;
            for (int i = 0; i < lane.points.size(); ++i) {
                const auto& p = lane.points[i];

                pcl::PointXYZ pt;
                pt.x = p.x; pt.y = p.y; pt.z = p.z;

                cloud->push_back(pt);
                cloud_meta.push_back({lane_id, i});
                current_global_index++;
            }
        }

        // 3. Deletion flags
        std::vector<bool> is_point_removed(total_points, false);

        // 4. KD-Tree setup
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // 5. Sort lanes by linearity and length
        std::sort(lane_metrics.begin(), lane_metrics.end(), [LINEARITY_TOLERANCE](const LaneMetric& a, const LaneMetric& b) {
            if (std::abs(a.linearity - b.linearity) > LINEARITY_TOLERANCE) {
                return a.linearity < b.linearity; // Higher linearity first
            }
            if (std::abs(a.length - b.length) > 1) {
                return a.length < b.length; // Longer length first
            }
            return a.id < b.id; // If equal, smaller ID first
        });

        // 6. Check overlapping points
        for (const auto& metric : lane_metrics) {
            int my_id = metric.id;
            const Lane& my_lane = global_map[my_id];
            int my_start_global_idx = lane_start_indices[my_id];
            int n_points = my_lane.points.size();

            int trim_front_idx = 0;
            int trim_back_idx = n_points;

            // Front trimming
            for (int i = 0; i < n_points; ++i) {
                int global_idx = my_start_global_idx + i;
                if (is_point_removed[global_idx]){
                    trim_front_idx++;
                    continue; // Already removed
                };
                std::vector<int> idxs;
                std::vector<float> dists;
                bool should_remove = false;

                if (kdtree.radiusSearch(cloud->points[global_idx], OVERLAP_RADIUS, idxs, dists) > 0) {
                    for (int neighbor_idx : idxs) {
                        int other_id = cloud_meta[neighbor_idx].lane_id;
                        if (other_id == my_id) continue; // Skip own lane

                        if (is_point_removed[neighbor_idx]) continue; // Already removed

                        should_remove = true;
                        break;
                    }
                }

                if (should_remove) {
                    is_point_removed[global_idx] = true;
                    trim_front_idx++;
                } else {
                    break; // Stop at first non-overlapping point
                }
            }

            // If fully trimmed before back trimming
            if(trim_front_idx >= trim_back_idx) continue;

            // Back trimming
            for (int i = n_points - 1; i >= trim_front_idx; --i) {
                int global_idx = my_start_global_idx + i;
                if (is_point_removed[global_idx]){
                    trim_back_idx--;
                    continue; // Already removed
                };

                std::vector<int> idxs;
                std::vector<float> dists;
                bool should_remove = false;

                if (kdtree.radiusSearch(cloud->points[global_idx], OVERLAP_RADIUS, idxs, dists) > 0) {
                    for (int neighbor_idx : idxs) {
                        int other_id = cloud_meta[neighbor_idx].lane_id;
                        if (other_id == my_id) continue; // Skip own lane

                        if (is_point_removed[neighbor_idx]) continue; // Already removed

                        should_remove = true;
                        break;
                    }
                }

                if (should_remove) {
                    is_point_removed[global_idx] = true;
                    trim_back_idx--;
                } else {
                    break; // Stop at first non-overlapping point
                }
            }

        }

        // 7. Remove empty lanes & trimmed points
        std::vector<int> empty_lanes;

        for (auto&pair : global_map) {
            int id = pair.first;
            Lane& lane = pair.second;
            int start_idx = lane_start_indices[id];

            std::vector<Point6D> survived_points;
            survived_points.reserve(lane.points.size());

            for (int i = 0; i < lane.points.size(); ++i) {
                int global_idx = start_idx + i;
                if (!is_point_removed[global_idx]) {
                    survived_points.push_back(lane.points[i]);
                }
            }

            lane.points = survived_points;

            if (lane.points.size() < 2) {
                empty_lanes.push_back(id);
            }
        }

        for (int id : empty_lanes) {
            global_map.erase(id);
        }
    }
}