#include "real_time_map/LaneClusterer.h" // 경로에 맞게 수정 필요
#include "toy_map_viewer/LaneUtils.h"
#include <queue>
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LaneClusterer::LaneClusterer(const Config& config) : config_(config) {}

std::vector<Lane> LaneClusterer::clusterAndSort(const std::map<int, Lane>& fragment_lanes) {
    std::vector<Lane> result_lanes;
    if (fragment_lanes.empty()) return result_lanes;

    // 1. Map -> Vector 변환 (포인터로 관리하여 복사 방지)
    std::vector<Lane*> lane_ptrs;
    for (auto& pair : fragment_lanes) {
        // const_cast는 map의 value가 const가 아니므로 필요 없으나, 
        // 입력이 const reference라면 복사본을 만들어야 할 수도 있음.
        // 여기서는 안전하게 복사본을 만들지 않고 처리하기 위해 const_cast를 쓰거나,
        // 혹은 인자를 복사해서 받아도 됨. 일단 const 제거를 위해 const_cast 사용 (주의)
        lane_ptrs.push_back(const_cast<Lane*>(&pair.second));
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
    // 단순 이중 루프로 모든 쌍 비교 (개수가 적으므로 O(N^2)도 허용 가능)
    // 최적화하려면 KD-Tree로 끝점만 넣어서 검색 가능
    for (size_t i = 0; i < lanes.size(); ++i) {
        for (size_t j = i + 1; j < lanes.size(); ++j) {
            if (isConnectable(*lanes[i], *lanes[j])) {
                adj_list[i].push_back(j);
                adj_list[j].push_back(i);
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
        // 여기서는 간단히 각 끝점의 dir 정보(dx,dy,dz)를 활용하거나 재계산.
        // Point6D에 dx,dy,dz가 이미 있다고 가정.
        
        Point6D dir1 = p1; // 복사
        Point6D dir2 = p2; 
        if (is_head1) { dir1.dx = -dir1.dx; dir1.dy = -dir1.dy; dir1.dz = -dir1.dz; } // Start점은 역방향이 나가는 방향
        // is_head2는 연결받는 쪽이므로 방향 그대로 둠? -> 아니요, 벡터 내적을 위해 방향 일치 여부 확인
        // 두 벡터가 "비슷한 방향"을 가리켜야 함 (이어지는 형상이면)
        
        // p1 -> p2 연결 벡터
        Point6D conn_vec;
        conn_vec.dx = p2.x - p1.x;
        conn_vec.dy = p2.y - p1.y;
        conn_vec.dz = p2.z - p1.z;
        double len = std::sqrt(conn_vec.dx*conn_vec.dx + conn_vec.dy*conn_vec.dy + conn_vec.dz*conn_vec.dz);
        if(len > 1e-6) { conn_vec.dx/=len; conn_vec.dy/=len; conn_vec.dz/=len; }

        // p1의 진행방향과 연결 벡터가 비슷해야 함
        double dot = dir1.dx * conn_vec.dx + dir1.dy * conn_vec.dy + dir1.dz * conn_vec.dz;
        double angle = std::acos(std::max(-1.0, std::min(1.0, dot))) * 180.0 / M_PI;
        
        return angle < config_.merge_angle_threshold;
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

    lane.points = sorted_points;
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

    for (size_t i = 0; i < points.size(); ++i) {
        std::vector<int> idxs;
        std::vector<float> sq_dists;
        // config_.point_connection_radius 반경 내 점들을 이웃으로 추가
        if (kdtree.radiusSearch(cloud->points[i], config_.point_connection_radius, idxs, sq_dists) > 0) {
            for (size_t k = 0; k < idxs.size(); ++k) {
                int neighbor_idx = idxs[k];
                if ((int)i == neighbor_idx) continue;

                double dist = std::sqrt(sq_dists[k]);
                graph[i].neighbors.push_back({neighbor_idx, dist});
            }
        }
    }
    return graph;
}

std::pair<int, int> LaneClusterer::findGraphEndpoints(const std::vector<GraphNode>& graph) {
    // 1. 임의의 점(0번)에서 가장 먼 점 u 찾기
    // 2. u에서 가장 먼 점 v 찾기
    // -> u, v가 endpoints

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

    // 그래프가 비어있지 않다고 가정
    // 단, 0번 노드가 고립된 점일 수 있으므로, 연결된 아무 점이나 찾아야 함
    // (여기서는 단순화를 위해 0번 시도, 만약 0번이 고립이면 다른 점 시도 필요)
    int u = get_farthest(0);
    if (u == -1) return {-1, -1}; // 0번이 고립됨

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