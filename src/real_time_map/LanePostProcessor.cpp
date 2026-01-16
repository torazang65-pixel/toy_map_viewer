#include "real_time_map/LanePostProcessor.h"
#include "toy_map_viewer/LaneUtils.h"
#include <cmath>
#include <algorithm>
#include <unordered_map>

LanePostProcessor::LanePostProcessor(const Config& config) : config_(config) {}

// Helper: 방향 벡터 계산
Point6D directionVector(const Point6D& from, const Point6D& to) {
    Point6D d;
    d.x = to.x - from.x;
    d.y = to.y - from.y;
    d.z = to.z - from.z;
    
    float len = std::sqrt(d.x*d.x + d.y*d.y + d.z*d.z);
    if (len > 1e-12) {
        d.x /= len; d.y /= len; d.z /= len;
    }
    return d; // dx, dy, dz 필드는 사용하지 않고 x,y,z를 벡터로 사용
}

// Helper: 점 간 거리 계산
float distSq(const Point6D& a, const Point6D& b) {
    return (a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y) + (a.z-b.z)*(a.z-b.z);
}

// Logic: post_processor.cpp의 getMergeCost 그대로 이식
std::optional<float> LanePostProcessor::getMergeCost(const Point6D& end1, const Point6D& prev1,
                                                     const Point6D& end2, const Point6D& prev2) {
    float d = std::sqrt(distSq(end1, end2));
    float d2 = std::sqrt(distSq(prev1, end2));
    float d3 = std::sqrt(distSq(end1, prev2));

    // 기하학적 조건 체크 (꼬임 방지)
    if (d >= d2 || d >= d3) return std::nullopt;
    if (d >= config_.merge_max_dist_th) return std::nullopt;
    
    // 아주 가까우면 비용 0
    if (d < config_.merge_min_dist_th) return 0.0f;

    // 각도 임계값 동적 계산
    //float angle_th = (config_.merge_max_dist_th - d) * config_.merge_max_angle_th 
    //               + (d - config_.merge_min_dist_th) * config_.merge_min_angle_th;
    float angle_th = config_.merge_max_angle_th;

    // 방향 벡터 계산
    Point6D v_conn = directionVector(end1, end2);
    Point6D v1 = directionVector(prev1, end1);
    Point6D v2 = directionVector(end2, prev2); // end2에서 prev2로 가는게 아니라, prev2->end2 방향이어야 자연스러움? 
    // 원본 코드: mean_dir = directionVector(prev1, end1) + directionVector(end2, prev2);
    // 주의: directionVector 구현에 따라 다름. 원본은 to - from.
    // 두 차선이 마주보고 연결되는 상황(Tail-Head)을 가정하면 방향이 일치해야 함.
    
    Point6D mean_dir;
    mean_dir.x = (v1.x + v2.x) * 0.5f;
    mean_dir.y = (v1.y + v2.y) * 0.5f;
    mean_dir.z = (v1.z + v2.z) * 0.5f;

    float cosTheta = mean_dir.x * v_conn.x + mean_dir.y * v_conn.y + mean_dir.z * v_conn.z;
    float angleDiff = std::acos(std::max(-1.0f, std::min(1.0f, cosTheta)));

    if (angleDiff < angle_th) {
        return angleDiff / angle_th;
    }
    return std::nullopt;
}

std::optional<LanePostProcessor::MergeCandidate> LanePostProcessor::checkMerge(const Lane& l1, const Lane& l2) {
    if (l1.points.size() < 2 || l2.points.size() < 2) return std::nullopt;

    const auto& p1_start = l1.points.front();
    const auto& p1_next  = l1.points[1];
    const auto& p1_end   = l1.points.back();
    const auto& p1_prev  = l1.points[l1.points.size()-2];

    const auto& p2_start = l2.points.front();
    const auto& p2_next  = l2.points[1];
    const auto& p2_end   = l2.points.back();
    const auto& p2_prev  = l2.points[l2.points.size()-2];

    // 4가지 경우의 수 (End-Start, End-End, Start-Start, Start-End)
    struct Case { Point6D e1, pr1, e2, pr2; int idx1, idx2; };
    std::vector<Case> cases = {
        {p1_end, p1_prev, p2_start, p2_next, (int)l1.points.size()-1, 0}, // Tail -> Head (Best Case)
        {p1_end, p1_prev, p2_end, p2_prev,   (int)l1.points.size()-1, (int)l2.points.size()-1},
        {p1_start, p1_next, p2_start, p2_next, 0, 0},
        {p1_start, p1_next, p2_end, p2_prev,   0, (int)l2.points.size()-1}
    };

    MergeCandidate best_cand;
    best_cand.cost = std::numeric_limits<float>::max();
    bool found = false;

    for(const auto& c : cases) {
        auto cost = getMergeCost(c.e1, c.pr1, c.e2, c.pr2);
        if(cost.has_value() && cost.value() < best_cand.cost) {
            best_cand.lane_id_1 = l1.id;
            best_cand.lane_id_2 = l2.id;
            best_cand.point_idx_1 = c.idx1;
            best_cand.point_idx_2 = c.idx2;
            best_cand.cost = cost.value();
            found = true;
        }
    }

    if(found) return best_cand;
    return std::nullopt;
}

Lane LanePostProcessor::mergeLanes(const Lane& l1, const Lane& l2, int idx1, int idx2) {
    Lane merged = l1;
    std::vector<Point6D> pts1 = l1.points;
    std::vector<Point6D> pts2 = l2.points;

    // 방향 맞추기
    if (idx1 == 0) std::reverse(pts1.begin(), pts1.end()); // Head가 연결점이면 뒤집음
    if (idx2 != 0) std::reverse(pts2.begin(), pts2.end()); // Tail이 연결점이면 뒤집음 (Head가 되도록)

    // pts1의 Tail과 pts2의 Head를 연결
    // 너무 가까우면 pts1의 마지막 점 제거 (중복 방지)
    if (!pts1.empty() && !pts2.empty()) {
        float d = std::sqrt(distSq(pts1.back(), pts2.front()));
        if (d < 1.0f) pts1.pop_back();
    }

    pts1.insert(pts1.end(), pts2.begin(), pts2.end());
    merged.points = pts1;
    return merged;
}

std::vector<Lane> LanePostProcessor::clusterAndSort(const std::map<int, Lane>& fragment_lanes) {
    std::map<int, Lane> current_lanes = fragment_lanes;

    // 1. 모든 가능한 병합 후보 계산
    std::vector<int> lane_ids;
    lane_ids.reserve(current_lanes.size());
    for (auto const& [id, lane] : current_lanes) lane_ids.push_back(id);

    std::vector<MergeCandidate> candidates;
    for (size_t i = 0; i < lane_ids.size(); ++i) {
        for (size_t j = i + 1; j < lane_ids.size(); ++j) {
            auto cand = checkMerge(current_lanes[lane_ids[i]], current_lanes[lane_ids[j]]);
            if (cand) candidates.push_back(cand.value());
        }
    }
    std::sort(candidates.begin(), candidates.end());

    // 2. post_processor 방식: endpoint 1:1 매칭 후 병합
    using EndPoint = std::pair<int, int>; // (lane_id, point_idx)
    struct EndPointHash {
        std::size_t operator()(const EndPoint& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    std::unordered_map<EndPoint, EndPoint, EndPointHash> merge_pair;
    merge_pair.reserve(candidates.size() * 2);

    for (const auto& cand : candidates) {
        EndPoint p1 = {cand.lane_id_1, cand.point_idx_1};
        EndPoint p2 = {cand.lane_id_2, cand.point_idx_2};
        if (merge_pair.count(p1) || merge_pair.count(p2)) continue;
        merge_pair[p1] = p2;
        merge_pair[p2] = p1;
    }

    while (!merge_pair.empty()) {
        auto it = merge_pair.begin();
        EndPoint p1 = it->first;
        EndPoint p2 = it->second;
        merge_pair.erase(p1);
        merge_pair.erase(p2);

        if (p1.first == p2.first) continue;

        int line1_id = p1.first;
        int line2_id = p2.first;
        int point1_idx = p1.second;
        int point2_idx = p2.second;

        if (!current_lanes.count(line1_id) || !current_lanes.count(line2_id)) continue;

        const Lane& line1 = current_lanes[line1_id];
        const Lane& line2 = current_lanes[line2_id];
        size_t line1_size = line1.points.size();
        size_t line2_size = line2.points.size();

        Lane merged = mergeLanes(line1, line2, point1_idx, point2_idx);
        current_lanes.erase(line2_id);
        current_lanes[line1_id] = merged;

        EndPoint merged_back_pt = {line1_id, static_cast<int>(merged.points.size() - 1)};
        EndPoint line1_front_pt = {line1_id, 0};
        EndPoint line1_back_pt = {line1_id, static_cast<int>(line1_size - 1)};
        EndPoint line2_front_pt = {line2_id, 0};
        EndPoint line2_back_pt = {line2_id, static_cast<int>(line2_size - 1)};

        // point1의 다른 merge pt를 위한 처리
        if (point1_idx == 0) {
            if (merge_pair.count(line1_back_pt)) {
                EndPoint target_pt = merge_pair[line1_back_pt];
                merge_pair[target_pt] = line1_front_pt;
                merge_pair[line1_front_pt] = target_pt;
            }
        }

        // point2의 다른 merge pt를 위한 처리
        if (point2_idx == 0) {
            if (merge_pair.count(line2_back_pt)) {
                EndPoint target_pt = merge_pair[line2_back_pt];
                merge_pair[target_pt] = merged_back_pt;
                merge_pair[merged_back_pt] = target_pt;
            }
        } else {
            if (merge_pair.count(line2_front_pt)) {
                EndPoint target_pt = merge_pair[line2_front_pt];
                merge_pair[target_pt] = merged_back_pt;
                merge_pair[merged_back_pt] = target_pt;
            }
        }
    }

    // 3. 결과 변환 및 Reordering (지그재그 펴기)
    std::vector<Lane> result;
    for (auto& pair : current_lanes) {
        Lane& lane = pair.second;
        
        // [Step 2] 지그재그 펴기 (Reordering & Smoothing)
        LaneUtils::ReorderPointsImproved(lane);

        // 너무 짧은 라인 제거 (옵션)
        if (LaneUtils::CalculateLaneLength(lane) >= config_.min_lane_length) {
            result.push_back(lane);
        }
    }

    return result;
}
