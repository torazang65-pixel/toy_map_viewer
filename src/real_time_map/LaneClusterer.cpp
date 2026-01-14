#include "real_time_map/LaneClusterer.h"
#include "toy_map_viewer/LaneUtils.h"
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LaneClusterer::LaneClusterer(const Config& config) : config_(config) {}

std::vector<Lane> LaneClusterer::clusterAndSort(const std::map<int, Lane>& fragment_lanes) {
    std::vector<Lane> result_lanes;
    if (fragment_lanes.empty()) return result_lanes;

    // 1. Map -> Vector 변환 (안전한 복사본 생성)
    std::vector<Lane> lanes_copy;
    lanes_copy.reserve(fragment_lanes.size());
    for (const auto& pair : fragment_lanes) {
        lanes_copy.push_back(pair.second);
    }

    std::vector<Lane*> lane_ptrs;
    lane_ptrs.reserve(lanes_copy.size());
    for (auto& lane : lanes_copy) {
        lane_ptrs.push_back(&lane);
    }

    // 2. Segment Clustering (Union-Find / BFS Grouping)
    std::vector<std::vector<int>> adj_list(lane_ptrs.size());
    buildSegmentGraph(lane_ptrs, adj_list);

    std::vector<bool> visited(lane_ptrs.size(), false);
    
    for (size_t i = 0; i < lane_ptrs.size(); ++i) {
        if (visited[i]) continue;

        // BFS로 연결된 그룹 찾기
        std::vector<Point6D> group_points;
        std::queue<int> q;
        q.push(i);
        visited[i] = true;

        while (!q.empty()) {
            int curr = q.front(); q.pop();
            // 그룹 내 모든 점 수집
            group_points.insert(group_points.end(), 
                                lane_ptrs[curr]->points.begin(), 
                                lane_ptrs[curr]->points.end());

            for (int neighbor : adj_list[curr]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    q.push(neighbor);
                }
            }
        }

        // 3. 그룹화된 점들로 새로운 Lane 생성 및 정렬
        if (group_points.size() > 2) {
            Lane merged_lane;
            merged_lane.id = merged_lane_id_counter_++;
            merged_lane.points = group_points;
            merged_lane.explicit_lane = true;
            merged_lane.valid = true;

            // [핵심] Geodesic Sort 적용
            sortPointsGeodesically(merged_lane);

            result_lanes.push_back(merged_lane);
        }
    }

    return result_lanes;
}

// --- [Step 1] Segment Clustering Logic ---

void LaneClusterer::buildSegmentGraph(const std::vector<Lane*>& lanes,
                                      std::vector<std::vector<int>>& adj_list) {
    if (lanes.empty()) return;

    // KD-Tree를 사용하여 O(N log N) 복잡도로 최적화
    // 각 Lane의 시작점과 끝점을 KD-Tree에 추가
    pcl::PointCloud<pcl::PointXYZ>::Ptr endpoints(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::pair<size_t, bool>> endpoint_info;  // {lane_idx, is_front}
    endpoint_info.reserve(lanes.size() * 2);

    for (size_t i = 0; i < lanes.size(); ++i) {
        if (lanes[i]->points.empty()) continue;

        const auto& front = lanes[i]->points.front();
        const auto& back = lanes[i]->points.back();

        endpoints->push_back(pcl::PointXYZ(
            static_cast<float>(front.x),
            static_cast<float>(front.y),
            static_cast<float>(front.z)));
        endpoint_info.push_back({i, true});

        endpoints->push_back(pcl::PointXYZ(
            static_cast<float>(back.x),
            static_cast<float>(back.y),
            static_cast<float>(back.z)));
        endpoint_info.push_back({i, false});
    }

    if (endpoints->empty()) return;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(endpoints);

    // 각 끝점에서 반경 내 다른 끝점 검색
    std::set<std::pair<size_t, size_t>> checked_pairs;  // 중복 검사 방지

    for (size_t ep_idx = 0; ep_idx < endpoints->size(); ++ep_idx) {
        size_t lane_i = endpoint_info[ep_idx].first;

        std::vector<int> neighbor_indices;
        std::vector<float> sq_distances;

        if (kdtree.radiusSearch(endpoints->points[ep_idx],
                                static_cast<float>(config_.merge_search_radius),
                                neighbor_indices, sq_distances) > 0) {

            for (int neighbor_idx : neighbor_indices) {
                size_t lane_j = endpoint_info[neighbor_idx].first;

                // 같은 Lane이면 스킵
                if (lane_i == lane_j) continue;

                // 이미 검사한 쌍이면 스킵
                auto pair_key = std::minmax(lane_i, lane_j);
                if (checked_pairs.count(pair_key)) continue;
                checked_pairs.insert(pair_key);

                // 실제 연결 가능 여부 검사
                if (isConnectable(*lanes[lane_i], *lanes[lane_j])) {
                    adj_list[lane_i].push_back(static_cast<int>(lane_j));
                    adj_list[lane_j].push_back(static_cast<int>(lane_i));
                }
            }
        }
    }
}

bool LaneClusterer::isConnectable(const Lane& l1, const Lane& l2) {
    if (l1.points.empty() || l2.points.empty()) return false;

    const Point6D& s1 = l1.points.front();
    const Point6D& e1 = l1.points.back();
    const Point6D& s2 = l2.points.front();
    const Point6D& e2 = l2.points.back();

    // 4가지 연결 조합 (Head-Head, Head-Tail, Tail-Head, Tail-Tail)
    // 중 하나라도 조건 만족하면 연결
    auto check = [&](const Point6D& p1, const Point6D& p2, bool is_head1, bool is_head2) {
        double dist = LaneUtils::GetDistance(p1, p2);
        if (dist > config_.merge_search_radius) return false;

        // 방향 벡터 계산 (p1의 나가는 방향 vs p2의 들어오는 방향)
        // Head면 역방향, Tail이면 정방향 벡터 사용

        // p1의 진행 방향 (나가는 방향)
        double dir1_dx = is_head1 ? -p1.dx : p1.dx;
        double dir1_dy = is_head1 ? -p1.dy : p1.dy;
        double dir1_dz = is_head1 ? -p1.dz : p1.dz;

        // p2의 들어오는 방향 (is_head2면 정방향, tail이면 역방향)
        double dir2_dx = is_head2 ? p2.dx : -p2.dx;
        double dir2_dy = is_head2 ? p2.dy : -p2.dy;
        double dir2_dz = is_head2 ? p2.dz : -p2.dz;

        // p1 -> p2 연결 벡터 (정규화)
        double conn_dx = p2.x - p1.x;
        double conn_dy = p2.y - p1.y;
        double conn_dz = p2.z - p1.z;
        double len = std::sqrt(conn_dx*conn_dx + conn_dy*conn_dy + conn_dz*conn_dz);

        if (len < 1e-6) {
            // 두 점이 거의 같은 위치 - 방향만 검사
            double dot_dirs = dir1_dx*dir2_dx + dir1_dy*dir2_dy + dir1_dz*dir2_dz;
            double angle_dirs = std::acos(std::clamp(dot_dirs, -1.0, 1.0)) * 180.0 / M_PI;
            return angle_dirs < config_.merge_angle_threshold;
        }

        conn_dx /= len;
        conn_dy /= len;
        conn_dz /= len;

        // p1의 진행방향과 연결 벡터가 비슷해야 함
        double dot1 = dir1_dx * conn_dx + dir1_dy * conn_dy + dir1_dz * conn_dz;
        double angle1 = std::acos(std::clamp(dot1, -1.0, 1.0)) * 180.0 / M_PI;

        // p2의 들어오는 방향과 연결 벡터(역방향)가 비슷해야 함
        double dot2 = dir2_dx * (-conn_dx) + dir2_dy * (-conn_dy) + dir2_dz * (-conn_dz);
        double angle2 = std::acos(std::clamp(dot2, -1.0, 1.0)) * 180.0 / M_PI;

        return angle1 < config_.merge_angle_threshold && angle2 < config_.merge_angle_threshold;
    };

    if (check(e1, s2, false, true)) return true; // Tail -> Head (가장 자연스러운 연결)
    if (check(s1, e2, true, false)) return true; // Head -> Tail
    if (check(e1, e2, false, false)) return true; // Tail -> Tail (방향이 서로 반대인 경우)
    if (check(s1, s2, true, true)) return true;   // Head -> Head

    return false;
}

// --- [Step 2] Geodesic Sorting Logic ---

void LaneClusterer::sortPointsGeodesically(Lane& lane) {
    if (lane.points.size() < 2) return;

    // 입력 검증: NaN 값이 있는 점 제거
    auto has_nan = [](const Point6D& pt) {
        return std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z);
    };

    size_t original_size = lane.points.size();
    lane.points.erase(
        std::remove_if(lane.points.begin(), lane.points.end(), has_nan),
        lane.points.end()
    );

    if (lane.points.size() != original_size) {
        std::cerr << "Warning: Lane " << lane.id << " had "
                  << (original_size - lane.points.size())
                  << " NaN points removed\n";
    }

    if (lane.points.size() < 2) return;

    // 1. 점들을 그래프로 변환
    auto graph = buildPointGraph(lane.points);
    if (graph.empty()) return;

    // 2. 그래프의 지름(Diameter)에 해당하는 양 끝점 찾기
    std::pair<int, int> endpoints = findGraphEndpoints(graph);
    int start_node = endpoints.first;
    // int end_node = endpoints.second; // 끝점은 확인용, 실제 정렬은 start 기준 거리로 함

    if (start_node == -1) return; // 그래프가 끊어져 있거나 오류

    // 3. 시작점 기준 Geodesic Distance 계산
    std::vector<double> distances = computeDistances(graph, start_node);

    // 4. 거리 순으로 정렬
    // (index, distance) 쌍을 만들어서 정렬
    std::vector<std::pair<double, int>> sort_helper;
    sort_helper.reserve(lane.points.size());
    for (size_t i = 0; i < lane.points.size(); ++i) {
        // 도달 불가능한 점(INF)은 맨 뒤로 보내거나 제외
        if (distances[i] == std::numeric_limits<double>::max()) continue;
        sort_helper.push_back({distances[i], static_cast<int>(i)});
    }

    std::sort(sort_helper.begin(), sort_helper.end());

    // 5. 정렬된 순서대로 Lane 재구성
    std::vector<Point6D> sorted_points;
    sorted_points.reserve(sort_helper.size());
    for (const auto& p : sort_helper) {
        sorted_points.push_back(lane.points[p.second]);
    }

    lane.points = std::move(sorted_points);
}

std::vector<LaneClusterer::GraphNode> LaneClusterer::buildPointGraph(const std::vector<Point6D>& points) {
    std::vector<GraphNode> graph(points.size());
    for(size_t i=0; i<points.size(); ++i) graph[i].id = i;

    // PCL KD-Tree 구성
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->resize(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        cloud->points[i].x = points[i].x;
        cloud->points[i].y = points[i].y;
        cloud->points[i].z = points[i].z;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    // [보완] Radius Search와 KNN Search를 함께 사용하여 연결성 보장
    // K=3 정도로 설정하여 자신을 제외한 가장 가까운 2개의 점과는 무조건 연결되도록 함
    const int K_NEIGHBORS = 3; 

    for (size_t i = 0; i < points.size(); ++i) {
        // 중복 엣지 추가를 방지하기 위해 이웃 ID를 set으로 관리 (선택 사항이나 권장)
        std::set<int> added_neighbors;

        // 1. Radius Search (기존 로직: 가까운 점들과 촘촘하게 연결)
        std::vector<int> idxs_radius;
        std::vector<float> sq_dists_radius;
        if (kdtree.radiusSearch(cloud->points[i], config_.point_connection_radius, idxs_radius, sq_dists_radius) > 0) {
            for (size_t k = 0; k < idxs_radius.size(); ++k) {
                int neighbor_idx = idxs_radius[k];
                if ((int)i == neighbor_idx) continue;

                if (added_neighbors.find(neighbor_idx) == added_neighbors.end()) {
                    double dist = std::sqrt(sq_dists_radius[k]);
                    graph[i].neighbors.push_back({neighbor_idx, dist});
                    added_neighbors.insert(neighbor_idx);
                }
            }
        }

        // 2. KNN Search (보완 로직: 거리가 멀어도 최소한의 연결 고리 생성)
        std::vector<int> idxs_knn;
        std::vector<float> sq_dists_knn;
        if (kdtree.nearestKSearch(cloud->points[i], K_NEIGHBORS, idxs_knn, sq_dists_knn) > 0) {
            for (size_t k = 0; k < idxs_knn.size(); ++k) {
                int neighbor_idx = idxs_knn[k];
                if ((int)i == neighbor_idx) continue;

                // 이미 Radius Search로 추가된 점은 패스
                if (added_neighbors.find(neighbor_idx) == added_neighbors.end()) {
                    double dist = std::sqrt(sq_dists_knn[k]);

                    if (dist > config_.merge_search_radius * 1.2) continue;

                    graph[i].neighbors.push_back({neighbor_idx, dist});
                    added_neighbors.insert(neighbor_idx);
                }
            }
        }
    }
    return graph;
}

std::pair<int, int> LaneClusterer::findGraphEndpoints(const std::vector<GraphNode>& graph) {
    // 1. 연결된 시작 노드 찾기
    // 2. 시작 노드에서 가장 먼 점 u 찾기
    // 3. u에서 가장 먼 점 v 찾기
    // -> u, v가 endpoints (그래프 지름의 양 끝점)

    if (graph.empty()) return {-1, -1};

    auto get_farthest = [&](int start_node) -> int {
        std::vector<double> dists = computeDistances(graph, start_node);
        int farthest_node = -1;
        double max_dist = -1.0;

        for (size_t i = 0; i < dists.size(); ++i) {
            if (dists[i] != std::numeric_limits<double>::max() && dists[i] > max_dist) {
                max_dist = dists[i];
                farthest_node = i;
            }
        }
        return farthest_node;
    };

    // 연결된 첫 번째 노드 찾기 (고립되지 않은 노드)
    int start = -1;
    for (size_t i = 0; i < graph.size(); ++i) {
        if (!graph[i].neighbors.empty()) {
            start = static_cast<int>(i);
            break;
        }
    }

    // 모든 노드가 고립됨
    if (start == -1) {
        // 최소 2개의 점이 있으면 첫 번째와 마지막을 반환
        if (graph.size() >= 2) {
            return {0, static_cast<int>(graph.size() - 1)};
        }
        return {-1, -1};
    }

    int u = get_farthest(start);
    if (u == -1) return {-1, -1};

    int v = get_farthest(u);
    return {u, v};
}

std::vector<double> LaneClusterer::computeDistances(const std::vector<GraphNode>& graph, int start_node) {
    std::vector<double> dist(graph.size(), std::numeric_limits<double>::max());
    
    // {cost, node_idx} (min-heap)
    using P = std::pair<double, int>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    dist[start_node] = 0.0;
    pq.push({0.0, start_node});

    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        if (d > dist[u]) continue;

        for (const auto& neighbor : graph[u].neighbors) {
            int v = neighbor.first;
            double weight = neighbor.second;

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }
    return dist;
}