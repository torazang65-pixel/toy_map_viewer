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
    double dot = dir1.dx * dir2.dx + dir1.dy * dir2.dy + dir1.dz * dir2.dz;
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * 180.0 / M_PI;
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

    std::vector<Lane> segments;
    for (const auto& pair : detected_lanes) {
        if (pair.second.points.size() < 2) continue;
        segments.push_back(pair.second);
    }
    if (segments.empty()) return merged_lanes_map;

    // Center-Out 정렬
    Point6D center = calculateBatchCenter(segments);
    std::sort(segments.begin(), segments.end(), [&](const Lane& a, const Lane& b) {
        double dist_a = LaneUtils::GetDistance(a.points[a.points.size()/2], center);
        double dist_b = LaneUtils::GetDistance(b.points[b.points.size()/2], center);
        return dist_a < dist_b;
    });

    std::vector<std::shared_ptr<Track>> active_tracks;

    for (const auto& segment : segments) {
        bool matched = false;
        int best_track_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        
        // 매칭 정보: [0]: Front에 붙음, [1]: Back에 붙음
        int match_type = 0; 
        bool should_reverse_segment = false; 

        const Point6D& seg_start = segment.points.front();
        const Point6D& seg_end = segment.points.back();
        
        // 세그먼트 방향 벡터
        Point6D seg_dir; 
        double len = LaneUtils::GetDistance(seg_start, seg_end);
        if(len < 1e-6) len = 1.0;
        seg_dir.dx = (seg_end.x - seg_start.x) / len;
        seg_dir.dy = (seg_end.y - seg_start.y) / len;
        seg_dir.dz = (seg_end.z - seg_start.z) / len;

        // 기존 트랙들과 매칭 시도
        for (int i = 0; i < active_tracks.size(); ++i) {
            auto& track = active_tracks[i];

            // ---------------------------------------------------
            // Case A: Front Extension (기존 트랙의 머리 + 새 세그먼트)
            // ---------------------------------------------------
            Point6D pred_f = track->predicted_front;
            Point6D trk_dir_f; // 트랙 머리 방향
            trk_dir_f.dx = track->state_front.at<float>(3);
            trk_dir_f.dy = track->state_front.at<float>(4);
            trk_dir_f.dz = track->state_front.at<float>(5);

            // A-1. 정방향 연결 (Track Head -> Seg Start)
            double d_f_normal = LaneUtils::GetDistance(pred_f, seg_start);
            // A-2. 역방향 연결 (Track Head -> Seg End) - 세그먼트가 뒤집혀 들어온 경우
            double d_f_reverse = LaneUtils::GetDistance(pred_f, seg_end);

            double d_front = d_f_normal;
            bool rev_front = false;
            if (d_f_reverse < d_f_normal) { d_front = d_f_reverse; rev_front = true; }

            // 각도 검사 (Front)
            Point6D check_dir_f = rev_front ? Point6D{-seg_dir.dx, -seg_dir.dy, -seg_dir.dz, 0,0,0} : seg_dir;
            if (getAngleDiff(trk_dir_f, check_dir_f) <= config_.match_angle_threshold) {
                if (d_front < config_.match_distance_threshold && d_front < min_dist) {
                    min_dist = d_front;
                    best_track_idx = i;
                    match_type = 1; // Front Match
                    should_reverse_segment = rev_front;
                }
            }

            // ---------------------------------------------------
            // Case B: Back Extension (새 세그먼트 + 기존 트랙의 꼬리)
            // ---------------------------------------------------
            Point6D pred_b = track->predicted_back;
            Point6D trk_dir_b; // 트랙 꼬리 방향 (역방향임에 주의)
            trk_dir_b.dx = track->state_back.at<float>(3);
            trk_dir_b.dy = track->state_back.at<float>(4);
            trk_dir_b.dz = track->state_back.at<float>(5);

            double d_b_normal = LaneUtils::GetDistance(pred_b, seg_end);
            
            // B-2. 역방향 연결 (Seg Start가 Track Tail과 만남) -> 세그먼트가 뒤집힘
            double d_b_reverse = LaneUtils::GetDistance(pred_b, seg_start);

            double d_back = d_b_normal;
            bool rev_back = false;
            if (d_b_reverse < d_b_normal) { d_back = d_b_reverse; rev_back = true; }

            // 각도 검사 (Back)
            Point6D check_dir_b = rev_back ? seg_dir : Point6D{-seg_dir.dx, -seg_dir.dy, -seg_dir.dz, 0,0,0};
            
            if (getAngleDiff(trk_dir_b, check_dir_b) <= config_.match_angle_threshold) {
                if (d_back < config_.match_distance_threshold && d_back < min_dist) {
                    min_dist = d_back;
                    best_track_idx = i;
                    match_type = 2; // Back Match
                    should_reverse_segment = rev_back;
                }
            }
        }

        // 매칭 처리
        if (best_track_idx != -1) {
            auto& track = active_tracks[best_track_idx];
            Lane to_merge = segment;
            
            // 세그먼트 뒤집기 적용
            if (should_reverse_segment) {
                std::reverse(to_merge.points.begin(), to_merge.points.end());
                seg_dir.dx = -seg_dir.dx; seg_dir.dy = -seg_dir.dy; seg_dir.dz = -seg_dir.dz;
            }

            if (match_type == 1) { 
                // [Front Merge]: Track ... -> New Segment
                // 측정치: 세그먼트의 '끝점'과 '방향'으로 Front KF 업데이트
                updateKF(track->kf_front, to_merge.points.back(), seg_dir);
                track->predicted_front = predictKF(track->kf_front);
                
                // 점 추가 (뒤에 붙임)
                track->lane.points.insert(track->lane.points.end(), 
                                          to_merge.points.begin(), to_merge.points.end());
            } 
            else if (match_type == 2) {
                // [Back Merge]: New Segment -> ... Track
                // 측정치: 세그먼트의 '시작점'과 '역방향'으로 Back KF 업데이트
                Point6D inv_seg_dir = {-seg_dir.dx, -seg_dir.dy, -seg_dir.dz, 0,0,0};
                updateKF(track->kf_back, to_merge.points.front(), inv_seg_dir);
                track->predicted_back = predictKF(track->kf_back);

                // 점 추가 (앞에 붙임)
                track->lane.points.insert(track->lane.points.begin(), 
                                          to_merge.points.begin(), to_merge.points.end());
            }

        } else {
            // New Track
            auto new_track = std::make_shared<Track>();
            new_track->id = track_id_counter_++;
            new_track->lane = segment;
            
            initializeTrack(*new_track, segment);
            active_tracks.push_back(new_track);
        }
    }

    for (const auto& track : active_tracks) {
        if (track->lane.points.size() < 2) continue;
        LaneUtils::ReorderPoints(track->lane);
        merged_lanes_map[track->id] = track->lane;
    }

    return merged_lanes_map;
}