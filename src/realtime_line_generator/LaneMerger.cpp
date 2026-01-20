#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <set>
#include <algorithm>
#include <iostream>
#include <Eigen/Dense>
#include <queue>
#include <deque>

#include "realtime_line_generator/LaneMerger.h"
#include "realtime_line_generator/LaneUtils.h"

namespace LaneMerger {
    // --- struct definitions ---
    struct EndPointNode {
        int lane_id;
        bool is_front; // true: start point, false: end point
        pcl::PointXYZ point;
        Eigen::Vector3d direction;
    };

    struct MergeCandidate {
        int id_a; bool is_front_a;
        int id_b; bool is_front_b;
        double score;

        bool operator<(const MergeCandidate& other) const {
            return score < other.score;
        }
    };
    struct Connection {
        int neighbor_id;
        bool my_end_is_front;     // 내 쪽 연결 부위 (Front/Back)
        bool neighbor_end_is_front; // 상대방 연결 부위 (Front/Back)
    };

    // --- Helper Functions ---
    void GetEndPointInfo(const Lane& lane, bool is_front, pcl::PointXYZ& out_pt, Eigen::Vector3d& out_dir) {
        if (lane.points.size() < 2) return;

        if (is_front) {
            // Point6D -> pcl::PointXYZ 수동 변환
            const auto& src = lane.points.front();
            out_pt.x = src.x;
            out_pt.y = src.y;
            out_pt.z = src.z;
        
        // 방향 계산
            out_dir[0] = lane.points[1].x - lane.points[0].x;
            out_dir[1] = lane.points[1].y - lane.points[0].y;
            out_dir[2] = lane.points[1].z - lane.points[0].z;
        } else {
            // Point6D -> pcl::PointXYZ 수동 변환
            const auto& src = lane.points.back();
            out_pt.x = src.x;
            out_pt.y = src.y;
            out_pt.z = src.z;

            size_t n = lane.points.size();
            out_dir[0] = lane.points[n-1].x - lane.points[n-2].x;
            out_dir[1] = lane.points[n-1].y - lane.points[n-2].y;
            out_dir[2] = lane.points[n-1].z - lane.points[n-2].z;
        }
        out_dir.normalize();
    }

    // --- MergeFragmentedLanes Implementation ---
    void MergeFragmentedLanes(std::map<int, Lane>& global_map, const LaneConfig& config) {
        if (global_map.empty()) return;

        std::cout << "[LaneMerger] Starting lane merging process..." << std::endl;

        const double SEARCH_RADIUS = config.search_radius; // 1.5 meter
        const double ANGLE_THRESHOLD_DEG = config.direction_threshold_deg;
        const double W_DIST = config.weight_distance;
        const double W_ANGLE = config.weight_direction;

        // 1. Forming KD-Tree for end points
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<EndPointNode> node_info;

        for (auto& pair : global_map) {
            const Lane& lane = pair.second;
            if (lane.points.size() < 2) continue;

            pcl::PointXYZ pt; Eigen::Vector3d dir;
            
            GetEndPointInfo(lane, true, pt, dir);
            cloud->push_back(pt);
            node_info.push_back({pair.first, true, pt, dir});

            GetEndPointInfo(lane, false, pt, dir);
            cloud->push_back(pt);
            node_info.push_back({pair.first, false, pt, dir});
        }

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloud);

        // 2. Finding merge candidates
        std::vector<MergeCandidate> candidates;
        std::set<std::pair<int, int>> visited_pairs;

        for (size_t i = 0; i < cloud->size(); ++i) {
            std::vector<int> idxs;
            std::vector<float> sq_dists;
            
            if (kdtree.radiusSearch(cloud->at(i), SEARCH_RADIUS, idxs, sq_dists) > 0) {
                for (size_t k = 0; k < idxs.size(); ++k) {
                    size_t j = idxs[k];
                    if (i == j) continue;

                    const auto& node_a = node_info[i];
                    const auto& node_b = node_info[j];

                    if (node_a.lane_id == node_b.lane_id) continue;

                    int min_id = std::min(node_a.lane_id, node_b.lane_id);
                    int max_id = std::max(node_a.lane_id, node_b.lane_id);
                    if (visited_pairs.count({min_id, max_id})) continue;

                    double dot = node_a.direction.dot(node_b.direction);
                    double abs_dot = std::abs(dot);
                    double angle_deg = std::acos(std::min(abs_dot, 1.0)) * 180.0 / M_PI;

                    if (angle_deg > ANGLE_THRESHOLD_DEG) continue;
                    
                    // ==============================================================
                    // new addition: 방향성 검사
                    // want to connect if the line that connects the two endpoints aligns with their directions
                    double alignment_threshold = std::cos(ANGLE_THRESHOLD_DEG * M_PI / 180.0);
                    Eigen::Vector3d conn_vec(node_b.point.x - node_a.point.x,
                                             node_b.point.y - node_a.point.y,
                                             node_b.point.z - node_a.point.z);
                    conn_vec.normalize();
                    double conn_dot_a = node_a.direction.dot(conn_vec);
                    double conn_dot_b = node_b.direction.dot(conn_vec);

                    bool ok_a = node_a.is_front ? (conn_dot_a < -alignment_threshold) : (conn_dot_a > alignment_threshold);
                    bool ok_b = node_b.is_front ? (conn_dot_b > alignment_threshold) : (conn_dot_b < -alignment_threshold);

                    // want to change the threshold values here using ANGLE_THRESHOLD_DEG
                    if(!ok_a || !ok_b) continue;
                    // ==============================================================
                    

                    double dist = std::sqrt(sq_dists[k]);
                    double score = (W_DIST * dist) + (W_ANGLE * (1.0 - abs_dot));

                    candidates.push_back({node_a.lane_id, node_a.is_front, 
                                          node_b.lane_id, node_b.is_front, score});
                }
            }
        }
        std::sort(candidates.begin(), candidates.end());

        //  3. Adjacency List 구성
        std::map<int, std::vector<Connection>> adj_list;
        std::map<int, bool> front_busy;
        std::map<int, bool> back_busy;

        for (const auto& cand : candidates) {
            bool a_busy = cand.is_front_a ? front_busy[cand.id_a] : back_busy[cand.id_a];
            bool b_busy = cand.is_front_b ? front_busy[cand.id_b] : back_busy[cand.id_b];

            if (a_busy || b_busy) continue; // 이미 더 좋은 짝과 연결됨

            // 연결 확정
            if (cand.is_front_a) front_busy[cand.id_a] = true; else back_busy[cand.id_a] = true;
            if (cand.is_front_b) front_busy[cand.id_b] = true; else back_busy[cand.id_b] = true;

            // 그래프에 양방향 추가
            adj_list[cand.id_a].push_back({cand.id_b, cand.is_front_a, cand.is_front_b});
            adj_list[cand.id_b].push_back({cand.id_a, cand.is_front_b, cand.is_front_a});
        }

        // 4. Merging Lanes based on connections
        std::map<int, Lane> new_global_map;
        std::set<int> processed_lanes;

        for (auto& pair : global_map) {
            int seed_id = pair.first;
            if(processed_lanes.count(seed_id)) continue;

            // BFS to find all connected lanes
            std::vector<int> group_nodes;
            std::queue<int> q;
            q.push(seed_id);
            std::set<int> visited_in_group;
            visited_in_group.insert(seed_id);

            int best_id = seed_id;
            double max_len = -1.0;

            while(!q.empty()) {
                int curr_id = q.front(); q.pop();
                group_nodes.push_back(curr_id);

                double curr_len = LaneUtils::CalculateLaneLength(global_map[curr_id]);
                if (curr_len > max_len) {
                    max_len = curr_len;
                    best_id = curr_id;
                }

                for (const auto& conn : adj_list[curr_id]) {
                    if (!visited_in_group.count(conn.neighbor_id)) {
                        visited_in_group.insert(conn.neighbor_id);
                        q.push(conn.neighbor_id);
                    }
                }
            }

            for(int id : group_nodes) {
                processed_lanes.insert(id);
            }

            std::deque<Point6D> merged_pts;
            const auto& root_pts = global_map[best_id].points;
            merged_pts.assign(root_pts.begin(), root_pts.end());
            
            // backward 연결된 Lane들 추가
            std::set<int> added_to_deque;
            added_to_deque.insert(best_id);

            int curr = best_id;
            bool curr_end_is_front = false; // Start from back end

            while(true) {
                int next_id = -1;
                bool next_end_is_front = false;

                if(adj_list.find(curr) != adj_list.end()) {
                    for (const auto& conn : adj_list[curr]) {
                        if (conn.my_end_is_front == curr_end_is_front && !added_to_deque.count(conn.neighbor_id)) {
                            next_id = conn.neighbor_id;
                            next_end_is_front = conn.neighbor_end_is_front;
                            break;
                        }
                    }
                }
                if (next_id == -1) break;

                const auto& pts = global_map[next_id].points;
                if (next_end_is_front) {
                    merged_pts.insert(merged_pts.end(), pts.begin(), pts.end());
                    curr_end_is_front = false;
                } else {
                    merged_pts.insert(merged_pts.end(), pts.rbegin(), pts.rend());
                    curr_end_is_front = true;
                }

                added_to_deque.insert(next_id);
                curr = next_id;
            }

            // forward 연결된 Lane들 추가
            curr = best_id;
            curr_end_is_front = true; // Start from front end

            while(true) {
                int next_id = -1;
                bool next_end_is_front = false;

                if(adj_list.find(curr) != adj_list.end()) {
                    for (const auto& conn : adj_list[curr]) {
                        if (conn.my_end_is_front == curr_end_is_front && !added_to_deque.count(conn.neighbor_id)) {
                            next_id = conn.neighbor_id;
                            next_end_is_front = conn.neighbor_end_is_front;
                            break;
                        }
                    }
                }
                if (next_id == -1) break;

                const auto& pts = global_map[next_id].points;
                if (next_end_is_front) {
                    merged_pts.insert(merged_pts.begin(), pts.rbegin(), pts.rend());
                    curr_end_is_front = false;
                } else {
                    merged_pts.insert(merged_pts.begin(), pts.begin(), pts.end());
                    curr_end_is_front = true;
                }

                added_to_deque.insert(next_id);
                curr = next_id;
            }

            // 5. Save merged lane
            Lane new_lane = global_map[best_id];
            new_lane.points.assign(merged_pts.begin(), merged_pts.end());
            new_global_map[best_id] = new_lane;
        }

        global_map = new_global_map;
        std::cout << "[LaneMerger] Merging completed." << std::endl;
        std::cout << " - Original lanes: " << global_map.size() + (processed_lanes.size() - new_global_map.size()) << std::endl;
        std::cout << " - Merged lanes: " << global_map.size() << std::endl;
    }
} // namespace LaneMerger