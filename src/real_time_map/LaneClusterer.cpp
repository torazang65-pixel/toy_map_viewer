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

            // [핵심] ReorderPointsImproved 적용 (MST 기반, O(N log N))
            LaneUtils::ReorderPointsImproved(merged_lane);

            // 최소 길이 필터링
            double lane_length = LaneUtils::CalculateLaneLength(merged_lane);
            if (lane_length >= config_.min_lane_length) {
                result_lanes.push_back(merged_lane);
            }
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

    // [강화된 조건 설정]
    const double LATERAL_THRESHOLD = 0.6; // 횡방향 오차 허용 범위 (m) - 차선 폭보다 훨씬 작게 설정 (보통 0.5~0.8m 추천)
    const double HEADING_THRESHOLD = 20.0; // 두 차선 자체의 각도 차이 허용 범위 (도)

    // 동적 각도 임계값 계산 헬퍼 함수
    auto getDynamicAngleThreshold = [&](double dist) -> double {
        const double MIN_DIST = config_.merge_min_dist_for_angle;  // 0.5m
        const double MAX_DIST = config_.merge_search_radius;       // 2.5m
        const double MIN_ANGLE = config_.merge_min_angle_threshold; // 5도 (가까울 때 엄격)
        const double MAX_ANGLE = config_.merge_angle_threshold;     // 10도 (멀 때 관대)

        if (dist <= MIN_DIST) return MIN_ANGLE;
        if (dist >= MAX_DIST) return MAX_ANGLE;

        // 선형 보간: 거리에 비례하여 각도 허용 범위 증가
        double ratio = (dist - MIN_DIST) / (MAX_DIST - MIN_DIST);
        return MIN_ANGLE + ratio * (MAX_ANGLE - MIN_ANGLE);
    };

    auto check = [&](const Point6D& p1, const Point6D& p2, bool is_head1, bool is_head2) {
        double dist = LaneUtils::GetDistance(p1, p2);
        if (dist > config_.merge_search_radius) return false;

        // 1. 방향 벡터 준비
        double dir1_dx = is_head1 ? -p1.dx : p1.dx;
        double dir1_dy = is_head1 ? -p1.dy : p1.dy;
        double dir1_dz = is_head1 ? -p1.dz : p1.dz;

        double dir2_dx = is_head2 ? p2.dx : -p2.dx;
        double dir2_dy = is_head2 ? p2.dy : -p2.dy;
        double dir2_dz = is_head2 ? p2.dz : -p2.dz;

        // 2. 연결 벡터 (P1 -> P2)
        double conn_dx = p2.x - p1.x;
        double conn_dy = p2.y - p1.y;
        double conn_dz = p2.z - p1.z;
        double len = std::sqrt(conn_dx*conn_dx + conn_dy*conn_dy + conn_dz*conn_dz);

        // 점이 거의 겹쳐있는 경우 (예외 처리)
        if (len < 1e-6) {
            double dot_dirs = dir1_dx*dir2_dx + dir1_dy*dir2_dy + dir1_dz*dir2_dz;
            double angle_dirs = std::acos(std::clamp(dot_dirs, -1.0, 1.0)) * 180.0 / M_PI;
            return angle_dirs < HEADING_THRESHOLD;
        }

        // 정규화된 연결 벡터
        double norm_conn_dx = conn_dx / len;
        double norm_conn_dy = conn_dy / len;
        double norm_conn_dz = conn_dz / len;

        // -----------------------------------------------------------
        // [조건 1] 횡방향 거리 (Lateral Distance) 검사 (핵심 추가 사항)
        // -----------------------------------------------------------
        // 두 벡터(dir1, conn)의 외적(Cross Product) 크기 = 평행사변형 넓이 = 밑변(1) * 높이(Lateral Dist)
        // dir1이 단위 벡터이므로, 외적의 크기가 곧 직선 P1->dir1 에서 P2까지의 수직 거리임.
        
        // 외적 계산 (Cross Product)
        double cross_x = dir1_dy * norm_conn_dz - dir1_dz * norm_conn_dy;
        double cross_y = dir1_dz * norm_conn_dx - dir1_dx * norm_conn_dz;
        double cross_z = dir1_dx * norm_conn_dy - dir1_dy * norm_conn_dx;
        double sin_val = std::sqrt(cross_x*cross_x + cross_y*cross_y + cross_z*cross_z);
        
        double lateral_dist = len * sin_val; // 빗변(len) * sin(theta) = 높이

        if (lateral_dist > LATERAL_THRESHOLD) return false; // 옆으로 너무 벗어남

        // -----------------------------------------------------------
        // [조건 2] 두 세그먼트 자체의 상대 각도 (Heading Alignment) 검사
        // -----------------------------------------------------------
        // 연결 부위가 자연스러워도(angle1, angle2 통과), 두 차선이 
        // 꺾여서 만나는(V자 형태) 경우를 방지하려면 두 방향 벡터 자체가 평행해야 함.
        
        double dot_dirs = dir1_dx*dir2_dx + dir1_dy*dir2_dy + dir1_dz*dir2_dz;
        double angle_dirs = std::acos(std::clamp(dot_dirs, -1.0, 1.0)) * 180.0 / M_PI;

        if (angle_dirs > HEADING_THRESHOLD) return false;

        // -----------------------------------------------------------
        // [조건 3] 거리 기반 동적 연결 각도 검사 (Smooth Connection)
        // -----------------------------------------------------------
        // 거리에 따라 각도 허용 범위 조정 (가까울수록 엄격, 멀수록 관대)
        double dynamic_angle_th = getDynamicAngleThreshold(dist);

        double dot1 = dir1_dx * norm_conn_dx + dir1_dy * norm_conn_dy + dir1_dz * norm_conn_dz;
        double angle1 = std::acos(std::clamp(dot1, -1.0, 1.0)) * 180.0 / M_PI;

        double dot2 = dir2_dx * norm_conn_dx + dir2_dy * norm_conn_dy + dir2_dz * norm_conn_dz;
        double angle2 = std::acos(std::clamp(dot2, -1.0, 1.0)) * 180.0 / M_PI;

        return angle1 < dynamic_angle_th && angle2 < dynamic_angle_th;
    };

    // 4가지 케이스 체크
    if (check(e1, s2, false, true)) return true; // Tail -> Head
    if (check(s1, e2, true, false)) return true; // Head -> Tail
    if (check(e1, e2, false, false)) return true; // Tail -> Tail
    if (check(s1, s2, true, true)) return true;   // Head -> Head

    return false;
}