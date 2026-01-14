#include "real_time_map/LaneTracker.h"
#include "toy_map_viewer/LaneUtils.h"
#include <algorithm>
#include <cmath>
#include <iostream>

LaneTracker::LaneTracker(const Config& config) : config_(config) {}

// KF 하나를 초기화하는 내부 헬퍼
static void initSingleKF(cv::KalmanFilter& kf, const Point6D& pt, const Point6D& dir, const LaneTracker::Config& cfg) {
    kf.init(6, 6, 0);
    
    // Transition Matrix (Position += Velocity)
    cv::setIdentity(kf.transitionMatrix);
    kf.transitionMatrix.at<float>(0, 3) = 1.0f;
    kf.transitionMatrix.at<float>(1, 4) = 1.0f;
    kf.transitionMatrix.at<float>(2, 5) = 1.0f;

    cv::setIdentity(kf.measurementMatrix);
    cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(cfg.process_noise));
    cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(cfg.measurement_noise));
    cv::setIdentity(kf.errorCovPost, cv::Scalar::all(0.1));

    // 초기 상태 설정
    kf.statePost.at<float>(0) = static_cast<float>(pt.x);
    kf.statePost.at<float>(1) = static_cast<float>(pt.y);
    kf.statePost.at<float>(2) = static_cast<float>(pt.z);
    kf.statePost.at<float>(3) = static_cast<float>(dir.dx);
    kf.statePost.at<float>(4) = static_cast<float>(dir.dy);
    kf.statePost.at<float>(5) = static_cast<float>(dir.dz);
}

// 방향성 기반 연결 지점 찾는 헬퍼 함수
static int findConnectionPoint(const std::vector<Point6D>& candidates, 
                               const Point6D& anchor_pt, 
                               const Point6D& track_dir, 
                               double angle_threshold) 
{
    
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& pt = candidates[i];
        
        // 기준점 -> 후보점 벡터 계산
        Point6D conn_vec;
        conn_vec.dx = pt.x - anchor_pt.x;
        conn_vec.dy = pt.y - anchor_pt.y;
        conn_vec.dz = pt.z - anchor_pt.z;
        
        // 거리가 너무 가까우면(0에 가까우면) 각도 계산이 불안정하므로 패스 (겹친 점으로 간주)
        double dist = std::sqrt(conn_vec.dx*conn_vec.dx + conn_vec.dy*conn_vec.dy + conn_vec.dz*conn_vec.dz);
        if (dist < 0.1) continue; 

        // 방향 정규화
        conn_vec.dx /= dist; conn_vec.dy /= dist; conn_vec.dz /= dist;

        // 각도 차이 계산 (내적)
        double dot = track_dir.dx * conn_vec.dx + track_dir.dy * conn_vec.dy + track_dir.dz * conn_vec.dz;
        // dot 값 클램핑 (-1.0 ~ 1.0)
        dot = std::max(-1.0, std::min(1.0, dot));
        double angle_deg = std::acos(dot) * 180.0 / M_PI;

        // 각도 차이가 임계값 이내이면, 이 점이 "자연스럽게 이어지는 시작점"임
        if (angle_deg < angle_threshold) {
            return static_cast<int>(i);
        }
    }
    
    return -1; // 연결 가능한 점을 찾지 못함 (완전 중복이거나 방향이 엉망)
}

void LaneTracker::initializeTrack(Track& track, const Lane& segment) {
    const Point6D& start = segment.points.front();
    const Point6D& end = segment.points.back();

    // 방향 벡터 계산 (Start -> End)
    Point6D dir;
    double len = LaneUtils::GetDistance(start, end);
    if(len < 1e-6) len = 1.0;
    dir.dx = (end.x - start.x) / len;
    dir.dy = (end.y - start.y) / len;
    dir.dz = (end.z - start.z) / len;

    // 1. Front KF 초기화 (위치: End, 속도: Dir) -> 앞으로 뻗어나감
    initSingleKF(track.kf_front, end, dir, config_);
    track.state_front = track.kf_front.statePost;
    track.predicted_front = predictKF(track.kf_front); // 첫 예측

    // 2. Back KF 초기화 (위치: Start, 속도: -Dir) -> 뒤로 뻗어나감
    Point6D inv_dir = {-dir.dx, -dir.dy, -dir.dz, 0,0,0};
    initSingleKF(track.kf_back, start, inv_dir, config_);
    track.state_back = track.kf_back.statePost;
    track.predicted_back = predictKF(track.kf_back);   // 첫 예측
}

Point6D LaneTracker::predictKF(cv::KalmanFilter& kf) {
    cv::Mat p = kf.predict();
    Point6D pt;
    pt.x = p.at<float>(0); pt.y = p.at<float>(1); pt.z = p.at<float>(2);
    pt.dx = p.at<float>(3); pt.dy = p.at<float>(4); pt.dz = p.at<float>(5);
    return pt;
}

void LaneTracker::updateKF(cv::KalmanFilter& kf, const Point6D& pt, const Point6D& dir) {
    cv::Mat meas(6, 1, CV_32F);
    meas.at<float>(0) = static_cast<float>(pt.x);
    meas.at<float>(1) = static_cast<float>(pt.y);
    meas.at<float>(2) = static_cast<float>(pt.z);
    meas.at<float>(3) = static_cast<float>(dir.dx);
    meas.at<float>(4) = static_cast<float>(dir.dy);
    meas.at<float>(5) = static_cast<float>(dir.dz);
    kf.correct(meas);
}

double LaneTracker::getAngleDiff(const Point6D& dir1, const Point6D& dir2) {
    double dot = dir1.dx * dir2.dx + dir1.dy * dir2.dy;
    double mag1 = std::sqrt(dir1.dx * dir1.dx + dir1.dy * dir1.dy);
    double mag2 = std::sqrt(dir2.dx * dir2.dx + dir2.dy * dir2.dy);
    if (mag1 < 1e-6 || mag2 < 1e-6) return 0.0;
    double val = dot / (mag1 * mag2);
    return std::acos(std::max(-1.0, std::min(1.0, val))) * 180.0 / M_PI;
}

Point6D LaneTracker::calculateBatchCenter(const std::vector<Lane>& lanes) {
    Point6D center = {0, 0, 0, 0, 0, 0};
    int count = 0;
    for(const auto& lane : lanes) {
        if(lane.points.empty()) continue;
        size_t mid = lane.points.size() / 2;
        center.x += lane.points[mid].x;
        center.y += lane.points[mid].y;
        center.z += lane.points[mid].z;
        count++;
    }
    if (count > 0) {
        center.x /= count; center.y /= count; center.z /= count;
    }
    return center;
}

std::map<int, Lane> LaneTracker::process(const std::map<int, Lane>& detected_lanes) {
    std::map<int, Lane> merged_lanes_map;
    if (detected_lanes.empty()) return merged_lanes_map;

    // 1. Map -> Vector 변환
    std::vector<Lane> segments;
    for (const auto& pair : detected_lanes) {
        if (pair.second.points.size() < 2) continue;
        segments.push_back(pair.second);
    }

    if (segments.empty()) return merged_lanes_map;

    // 2. Center-Out Strategy: 배치의 중심에서 가까운 순서대로 정렬
    Point6D center = calculateBatchCenter(segments);
    std::sort(segments.begin(), segments.end(), [&](const Lane& a, const Lane& b) {
        double dist_a = LaneUtils::GetDistance(a.points[a.points.size()/2], center);
        double dist_b = LaneUtils::GetDistance(b.points[b.points.size()/2], center);
        return dist_a < dist_b;
    });

    // 3. Tracking Loop
    std::vector<std::shared_ptr<Track>> active_tracks;

    for (const auto& segment : segments) {
        bool matched = false;
        int best_track_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        int match_type = 0; // 0: None, 1: Front, 2: Back
        bool should_reverse_segment = false; 

        const Point6D& seg_start = segment.points.front();
        const Point6D& seg_end = segment.points.back();
        
        // 세그먼트 방향 벡터 (Start -> End)
        Point6D seg_dir; 
        double len = LaneUtils::GetDistance(seg_start, seg_end);
        if(len < 1e-6) len = 1.0;
        seg_dir.dx = (seg_end.x - seg_start.x) / len;
        seg_dir.dy = (seg_end.y - seg_start.y) / len;
        seg_dir.dz = (seg_end.z - seg_start.z) / len;
        
        // 세그먼트 역방향 벡터
        Point6D seg_dir_inv = {-seg_dir.dx, -seg_dir.dy, -seg_dir.dz, 0,0,0};

        for (int i = 0; i < active_tracks.size(); ++i) {
            auto& track = active_tracks[i];

            // ===================================================
            // Case A: Front Extension (기존 트랙 머리 + 새 세그먼트)
            // ===================================================
            Point6D pred_f = track->predicted_front;
            Point6D trk_dir_f; 
            trk_dir_f.dx = track->state_front.at<float>(3);
            trk_dir_f.dy = track->state_front.at<float>(4);
            trk_dir_f.dz = track->state_front.at<float>(5);

            double d_f_normal = LaneUtils::GetDistance(pred_f, seg_start);
            double d_f_reverse = LaneUtils::GetDistance(pred_f, seg_end);

            bool is_front_reverse = (d_f_reverse < d_f_normal);
            double d_front = is_front_reverse ? d_f_reverse : d_f_normal;
            Point6D target_pt_f = is_front_reverse ? seg_end : seg_start;

            double conn_dx_f = target_pt_f.x - pred_f.x;
            double conn_dy_f = target_pt_f.y - pred_f.y;
            double dot_f = conn_dx_f * trk_dir_f.dx + conn_dy_f * trk_dir_f.dy;

            // -0.5m 정도의 약간의 겹침은 허용하지만, 너무 뒤쪽이면 스킵
            if (dot_f > -0.5) {
                // 비교: 트랙 방향 vs (세그먼트가 연결되면 뻗어나갈 방향)
                Point6D check_dir_f = is_front_reverse ? seg_dir_inv : seg_dir;
                
                if (getAngleDiff(trk_dir_f, check_dir_f) <= config_.match_angle_threshold) {
                    if (d_front < config_.match_distance_threshold && d_front < min_dist) {
                        min_dist = d_front;
                        best_track_idx = i;
                        match_type = 1; // Front Match
                        should_reverse_segment = is_front_reverse;
                    }
                }
            }

            // ===================================================
            // Case B: Back Extension (새 세그먼트 + 기존 트랙 꼬리)
            // ===================================================
            Point6D pred_b = track->predicted_back;
            Point6D trk_dir_b; // 트랙 꼬리에서 '뒤로' 나가는 방향
            trk_dir_b.dx = track->state_back.at<float>(3);
            trk_dir_b.dy = track->state_back.at<float>(4);
            trk_dir_b.dz = track->state_back.at<float>(5);

            double d_b_normal = LaneUtils::GetDistance(pred_b, seg_end);
            double d_b_reverse = LaneUtils::GetDistance(pred_b, seg_start);

            bool is_back_reverse = (d_b_reverse < d_b_normal);
            double d_back = is_back_reverse ? d_b_reverse : d_b_normal;
            Point6D target_pt_b = is_back_reverse ? seg_start : seg_end;

            double conn_dx_b = target_pt_b.x - pred_b.x;
            double conn_dy_b = target_pt_b.y - pred_b.y;
            double dot_b = conn_dx_b * trk_dir_b.dx + conn_dy_b * trk_dir_b.dy;

            if (dot_b > -0.5) {
                Point6D check_dir_b = is_back_reverse ? seg_dir : seg_dir_inv;

                if (getAngleDiff(trk_dir_b, check_dir_b) <= config_.match_angle_threshold) {
                    if (d_back < config_.match_distance_threshold && d_back < min_dist) {
                        min_dist = d_back;
                        best_track_idx = i;
                        match_type = 2; // Back Match
                        should_reverse_segment = is_back_reverse;
                    }
                }
            }
        }

        if (best_track_idx != -1) {
            auto& track = active_tracks[best_track_idx];
            Lane to_merge = segment;
            
            // 세그먼트 뒤집기 적용
            if (should_reverse_segment) {
                std::reverse(to_merge.points.begin(), to_merge.points.end());
            }

            // [사용자 요청 로직 적용] 
            // "이어지는 각도가 threshold 안에 들어올 때까지 찾고, 그 이전은 삭제"
            const double SMOOTH_CONNECTION_ANGLE = 30.0; // 연결 허용 각도 (약간 여유 있게)

            if (match_type == 1) { // [Front Merge]: Track(End) -> New Seg
                Point6D meas_dir = should_reverse_segment ? seg_dir_inv : seg_dir;
                updateKF(track->kf_front, to_merge.points.back(), meas_dir);
                track->predicted_front = predictKF(track->kf_front);
                
                if (!track->lane.points.empty()) {
                    Point6D last_pt = track->lane.points.back();
                    
                    // Track의 진행 방향 (Front KF 상태)
                    Point6D trk_dir;
                    trk_dir.dx = track->state_front.at<float>(3);
                    trk_dir.dy = track->state_front.at<float>(4);
                    trk_dir.dz = track->state_front.at<float>(5);

                    // 유효한 연결 점 찾기
                    int valid_idx = findConnectionPoint(to_merge.points, last_pt, trk_dir, SMOOTH_CONNECTION_ANGLE);

                    if (valid_idx != -1) {
                        // 찾았으면 그 점부터 끝까지 병합 (이전 점들은 중복/노이즈로 간주하여 삭제됨)
                        track->lane.points.insert(track->lane.points.end(), 
                                                  to_merge.points.begin() + valid_idx, 
                                                  to_merge.points.end());
                    } else {
                        // 못 찾았으면? -> "해당 segment는 전부 중복된거라고 판단하고 없애버림"
                        // (아무것도 하지 않음)
                    }
                } else {
                    // 트랙이 비어있었다면 그냥 추가 (예외 처리)
                    track->lane.points = to_merge.points;
                }

            } else if (match_type == 2) { // [Back Merge]: New Seg -> Track(Start)
                Point6D meas_dir = should_reverse_segment ? seg_dir : seg_dir_inv;
                updateKF(track->kf_back, to_merge.points.front(), meas_dir);
                track->predicted_back = predictKF(track->kf_back);

                if (!track->lane.points.empty()) {
                    Point6D first_pt = track->lane.points.front();

                    // Track의 역방향 진행 방향 (Back KF 상태)
                    Point6D trk_back_dir;
                    trk_back_dir.dx = track->state_back.at<float>(3);
                    trk_back_dir.dy = track->state_back.at<float>(4);
                    trk_back_dir.dz = track->state_back.at<float>(5);

                    // Back Merge는 세그먼트의 '뒤'에서부터 '앞'으로 탐색해야 함
                    // (세그먼트의 끝점이 트랙의 시작점과 가까우므로, 끝점부터 역순으로 검사)
                    std::vector<Point6D> reversed_points = to_merge.points;
                    std::reverse(reversed_points.begin(), reversed_points.end());

                    int valid_idx = findConnectionPoint(reversed_points, first_pt, trk_back_dir, SMOOTH_CONNECTION_ANGLE);

                    if (valid_idx != -1) {
                        // 찾았으면 유효한 점들을 추출
                        // reversed_points[valid_idx]가 유효한 점이므로, 
                        // 원래 순서에서는 (size - 1 - valid_idx) 인덱스가 됨.
                        // 이 점을 포함하여 그 '앞쪽' 점들을 가져와야 함.
                        
                        // 예: Orig [A, B, C, D], valid_idx(from D) = 1 (C가 유효)
                        // -> [A, B, C]를 가져와야 함.
                        
                        auto start_iter = to_merge.points.begin();
                        auto end_iter = to_merge.points.end() - valid_idx; 
                        
                        // 트랙의 앞에 삽입
                        track->lane.points.insert(track->lane.points.begin(), 
                                                  start_iter, end_iter);
                    } else {
                        // 전부 중복 -> 삭제 (스킵)
                    }
                } else {
                    track->lane.points = to_merge.points;
                }
            }

        } else {
            // New Track 생성
            auto new_track = std::make_shared<Track>();
            new_track->id = track_id_counter_++;
            new_track->lane = segment;
            
            initializeTrack(*new_track, segment);
            active_tracks.push_back(new_track);
        }
    }

    // 4. 결과 반환 및 후처리
    for (const auto& track : active_tracks) {
        if (track->lane.points.size() < 2) continue; 
        
        // 최종적으로 엉킨 점 풀기 (Reorder)
        LaneUtils::ReorderPoints(track->lane);
        
        merged_lanes_map[track->id] = track->lane;
    }

    return merged_lanes_map;
}