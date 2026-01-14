#include "real_time_map/LaneTracker.h"
#include "toy_map_viewer/LaneUtils.h"
#include <algorithm>
#include <cmath>
#include <iostream>

LaneTracker::LaneTracker(const Config& config) : config_(config) {}

void LaneTracker::initializeKF(Track& track, const Point6D& start_pt, const Point6D& dir) {
    // 6 State (x, y, z, vx, vy, vz), 6 Measurement
    track.kf.init(6, 6, 0);
    track.meas = cv::Mat::zeros(6, 1, CV_32F);

    // Transition Matrix (F) - 등속 운동 모델 가정
    // P_next = P_curr + V_curr * dt (dt=1로 가정, 공간적 step)
    cv::setIdentity(track.kf.transitionMatrix);
    track.kf.transitionMatrix.at<float>(0, 3) = 1.0f;
    track.kf.transitionMatrix.at<float>(1, 4) = 1.0f;
    track.kf.transitionMatrix.at<float>(2, 5) = 1.0f;

    // Measurement Matrix (H) - 모든 상태를 관측함
    cv::setIdentity(track.kf.measurementMatrix);

    // Covariance Matrices
    cv::setIdentity(track.kf.processNoiseCov, cv::Scalar::all(config_.process_noise));
    cv::setIdentity(track.kf.measurementNoiseCov, cv::Scalar::all(config_.measurement_noise));
    cv::setIdentity(track.kf.errorCovPost, cv::Scalar::all(0.1));

    // Initial State Setting
    track.kf.statePost.at<float>(0) = static_cast<float>(start_pt.x);
    track.kf.statePost.at<float>(1) = static_cast<float>(start_pt.y);
    track.kf.statePost.at<float>(2) = static_cast<float>(start_pt.z);
    track.kf.statePost.at<float>(3) = static_cast<float>(dir.dx);
    track.kf.statePost.at<float>(4) = static_cast<float>(dir.dy);
    track.kf.statePost.at<float>(5) = static_cast<float>(dir.dz);

    track.state = track.kf.statePost;
    track.predicted_head = start_pt; // 초기 예측값은 시작점
}

Point6D LaneTracker::predictKF(Track& track) {
    cv::Mat p = track.kf.predict();
    Point6D pt;
    pt.x = p.at<float>(0);
    pt.y = p.at<float>(1);
    pt.z = p.at<float>(2);
    pt.dx = p.at<float>(3);
    pt.dy = p.at<float>(4);
    pt.dz = p.at<float>(5);
    return pt;
}

void LaneTracker::updateKF(Track& track, const Point6D& measurement_pt, const Point6D& measurement_dir) {
    track.meas.at<float>(0) = static_cast<float>(measurement_pt.x);
    track.meas.at<float>(1) = static_cast<float>(measurement_pt.y);
    track.meas.at<float>(2) = static_cast<float>(measurement_pt.z);
    track.meas.at<float>(3) = static_cast<float>(measurement_dir.dx);
    track.meas.at<float>(4) = static_cast<float>(measurement_dir.dy);
    track.meas.at<float>(5) = static_cast<float>(measurement_dir.dz);

    track.kf.correct(track.meas);
    
    // 보정된 상태를 가져와서 예측 헤드 업데이트 (다음 매칭을 위해)
    // 실제로는 predict()가 다음 상태를 주지만, 시각적으로 연결을 위해 현재 상태 유지
}

double LaneTracker::getAngleDiff(const Point6D& dir1, const Point6D& dir2) {
    double dot = dir1.dx * dir2.dx + dir1.dy * dir2.dy + dir1.dz * dir2.dz;
    // clip to [-1, 1] to avoid NaN in acos
    dot = std::max(-1.0, std::min(1.0, dot));
    return std::acos(dot) * 180.0 / M_PI;
}

Point6D LaneTracker::calculateBatchCenter(const std::vector<Lane>& lanes) {
    Point6D center = {0, 0, 0, 0, 0, 0};
    int count = 0;
    for(const auto& lane : lanes) {
        if(lane.points.empty()) continue;
        // 세그먼트의 중점을 샘플로 사용
        size_t mid = lane.points.size() / 2;
        center.x += lane.points[mid].x;
        center.y += lane.points[mid].y;
        center.z += lane.points[mid].z;
        count++;
    }
    if (count > 0) {
        center.x /= count;
        center.y /= count;
        center.z /= count;
    }
    return center;
}

std::map<int, Lane> LaneTracker::process(const std::map<int, Lane>& detected_lanes) {
    std::map<int, Lane> merged_lanes_map;
    if (detected_lanes.empty()) return merged_lanes_map;

    // 1. Map -> Vector 변환 및 유효성 검사
    std::vector<Lane> segments;
    for (const auto& pair : detected_lanes) {
        if (pair.second.points.size() < 2) continue;
        segments.push_back(pair.second);
    }

    if (segments.empty()) return merged_lanes_map;

    // 2. Center-Out Strategy: 배치의 중심 계산 및 거리순 정렬
    // (차량 위치를 모르므로 데이터 분포의 무게중심을 사용)
    Point6D center = calculateBatchCenter(segments);
    
    std::sort(segments.begin(), segments.end(), [&](const Lane& a, const Lane& b) {
        double dist_a = LaneUtils::GetDistance(a.points[a.points.size()/2], center);
        double dist_b = LaneUtils::GetDistance(b.points[b.points.size()/2], center);
        return dist_a < dist_b; // 중심에 가까운 순서대로
    });

    // 3. Tracking Loop
    std::vector<std::shared_ptr<Track>> active_tracks;

    for (const auto& segment : segments) {
        bool matched = false;
        int best_track_idx = -1;
        double min_dist = std::numeric_limits<double>::max();
        bool should_reverse = false; // 세그먼트 방향이 반대인 경우 뒤집기 위해

        // Segment 정보 추출
        const Point6D& seg_start = segment.points.front();
        const Point6D& seg_end = segment.points.back();
        // 방향 벡터 (Normalize)
        Point6D seg_dir; 
        double len = LaneUtils::GetDistance(seg_start, seg_end);
        seg_dir.dx = (seg_end.x - seg_start.x) / len;
        seg_dir.dy = (seg_end.y - seg_start.y) / len;
        seg_dir.dz = (seg_end.z - seg_start.z) / len;

        // 기존 트랙들과 매칭 시도
        for (int i = 0; i < active_tracks.size(); ++i) {
            auto& track = active_tracks[i];
            
            // KF 예측 (다음 위치)
            Point6D predicted = track->predicted_head;
            
            // 1. 거리 체크 (예측 지점 vs 세그먼트 시작점)
            // 정방향 연결 시도
            double dist_forward = LaneUtils::GetDistance(predicted, seg_start);
            // 역방향 연결 시도 (세그먼트가 반대로 들어온 경우)
            double dist_reverse = LaneUtils::GetDistance(predicted, seg_end);

            double dist = dist_forward;
            bool is_reverse = false;

            if (dist_reverse < dist_forward) {
                dist = dist_reverse;
                is_reverse = true;
            }

            if (dist > config_.match_distance_threshold) continue;

            // 2. 방향(Angle) 체크 (X자 교차 방지)
            // 트랙의 진행 방향
            Point6D track_dir;
            track_dir.dx = track->state.at<float>(3);
            track_dir.dy = track->state.at<float>(4);
            track_dir.dz = track->state.at<float>(5);

            // 세그먼트 방향 (역방향이면 뒤집어서 비교)
            Point6D check_seg_dir = is_reverse ? 
                                    Point6D{-seg_dir.dx, -seg_dir.dy, -seg_dir.dz, 0,0,0} : seg_dir;

            double angle_diff = getAngleDiff(track_dir, check_seg_dir);

            if (angle_diff > config_.match_angle_threshold) continue; // 각도 차이가 크면 다른 차선임 (X자)

            // 최적 매칭 갱신
            if (dist < min_dist) {
                min_dist = dist;
                best_track_idx = i;
                should_reverse = is_reverse;
            }
        }

        // 매칭 결과 처리
        if (best_track_idx != -1) {
            // 매칭 성공 -> KF Update & Merge
            auto& track = active_tracks[best_track_idx];
            
            // 세그먼트 복사 (필요시 뒤집기)
            Lane to_merge = segment;
            if (should_reverse) {
                std::reverse(to_merge.points.begin(), to_merge.points.end());
                // 방향 벡터도 반대로
                seg_dir.dx = -seg_dir.dx; seg_dir.dy = -seg_dir.dy; seg_dir.dz = -seg_dir.dz;
            }

            // KF Update
            // 측정치: 세그먼트의 '끝점'과 '방향'을 사용하여 상태 업데이트
            // (시작점은 이미 연결되었으므로, 관성을 끝점 기준으로 갱신)
            updateKF(*track, to_merge.points.back(), seg_dir);
            track->predicted_head = predictKF(*track); // 다음 스텝 예측

            // 점 추가 (KF로 보정된 좌표를 쓸 수도 있지만, 여기서는 원본 데이터를 붙임)
            // KF는 '연결 여부'와 '방향성 유지'에 주로 기여
            track->lane.points.insert(track->lane.points.end(), 
                                      to_merge.points.begin(), 
                                      to_merge.points.end());
        } else {
            // 매칭 실패 -> 새로운 트랙 생성 (New Born)
            auto new_track = std::make_shared<Track>();
            new_track->id = track_id_counter_++;
            new_track->lane = segment;
            
            // KF 초기화 (시작점이 아니라, 확장 방향인 끝점을 기준으로 상태 설정)
            // 초기 위치: 끝점, 초기 속도: 방향
            initializeKF(*new_track, segment.points.back(), seg_dir);
            new_track->predicted_head = predictKF(*new_track); // 첫 예측

            active_tracks.push_back(new_track);
        }
    }

    // 4. 결과 반환
    for (const auto& track : active_tracks) {
        // 너무 짧은 트랙 제거 로직 등을 여기에 추가 가능
        if (track->lane.points.size() < 2) continue; 
        
        // 최종적으로 정렬 한 번 더 수행 (깔끔하게)
        LaneUtils::ReorderPoints(track->lane);
        merged_lanes_map[track->id] = track->lane;
    }

    return merged_lanes_map;
}