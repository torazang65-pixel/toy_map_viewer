#include <ros/ros.h>
#include <ros/package.h>
#include <toy_map_viewer/realtime_line_builder.h>
#include "real_time_map/FrameLoader.h"
#include <filesystem> // C++17 필욕
#include <vector>
#include <algorithm>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_builder_node");
    ros::NodeHandle nh("~");

    // 1. 기본 설정 및 로더 초기화
    FrameLoader loader(nh);
    toy_map_viewer::RealTimeLineBuilder builder(nh);

    // 2. 데이터 경로 파악 (FrameLoader의 경로와 일치해야 함)
    std::string date;
    nh.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2");
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    std::string frames_path = pkg_path + "/data/lane_change_data_converted/Raw/" + date + "/frames/";

    // 3. 디렉토리 스캔을 통해 프레임 인덱스 수집
    std::vector<int> frame_indices;
    try {
        if (fs::exists(frames_path) && fs::is_directory(frames_path)) {
            for (const auto& entry : fs::directory_iterator(frames_path)) {
                std::string filename = entry.path().filename().string();
                // 파일 형식이 'frame_N.bin' 또는 'N.bin' 인 경우 숫자만 추출
                // 예: frame_123.bin -> 123
                if (filename.find("frame_") != std::string::npos) {
                    size_t start = filename.find("_") + 1;
                    size_t end = filename.find(".bin");
                    if (start != std::string::npos && end != std::string::npos) {
                        frame_indices.push_back(std::stoi(filename.substr(start, end - start)));
                    }
                }
            }
        }
    } catch (const fs::filesystem_error& e) {
        ROS_ERROR("Filesystem error: %s", e.what());
        return 1;
    }

    if (frame_indices.empty()) {
        ROS_ERROR("No frames found in directory: %s", frames_path.c_str());
        return 1;
    }

    // 4. 프레임 번호 정렬 (순차 처리를 위해)
    std::sort(frame_indices.begin(), frame_indices.end());
    int total_frames = frame_indices.size();

    // 5. 재생 속도 설정
    double loop_rate_hz;
    nh.param("playback_rate", loop_rate_hz, 10.0);
    ros::Rate loop_rate(loop_rate_hz);

    ROS_INFO("[RealTimeBuilder] Auto-detected %d frames in %s", total_frames, frames_path.c_str());
    ROS_INFO("[RealTimeBuilder] Processing from frame %d to %d", frame_indices.front(), frame_indices.back());

    // 6. 감지된 모든 프레임 순회
    for (int current_frame_idx : frame_indices) {
        if (!ros::ok()) break;

        // 파일 로드
        auto cloud = loader.loadFrame(current_frame_idx);

        if (cloud != nullptr) {
            // PCL 형식을 빌더 내부 Point 형식으로 변환
            std::vector<linemapdraft_builder::data_types::Point> frame_points;
            frame_points.reserve(cloud->size());

            for (const auto& pcl_pt : *cloud) {
                linemapdraft_builder::data_types::Point p;
                p.x = pcl_pt.x;
                p.y = pcl_pt.y;
                p.z = pcl_pt.z;
                p.yaw = pcl_pt.intensity;
                p.density = static_cast<uint32_t>(pcl_pt.intensity);
                p.polyline_id = linemapdraft_builder::data_types::Unclassified;
                frame_points.push_back(p);
            }

            // 실시간 파이프라인 실행
            builder.processPipeline(frame_points, loader.getFrameId().c_str());

            ROS_INFO("Processed frame %d (%lu points)", current_frame_idx, frame_points.size());
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[RealTimeBuilder] All detected frames processed.");
    return 0;
}