#include <ros/ros.h>
#include <toy_map_viewer/realtime_line_builder.h>
#include "real_time_map/FrameLoader.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "realtime_builder_node");
    ros::NodeHandle nh("~");

    FrameLoader loader(nh);
    toy_map_viewer::RealTimeLineBuilder builder(nh);

    // 2. 재생 루프 제어 파라미터
    int start_frame, end_frame;
    double loop_rate_hz;
    nh.param("start_frame", start_frame, 0); // 루프를 시작할 프레임 번호
    nh.param("end_frame", end_frame, 299);     // 루프를 종료할 프레임 번호
    nh.param("playback_rate", loop_rate_hz, 10.0); // 재생 속도 (Hz)

    ros::Rate loop_rate(loop_rate_hz);
    int current_frame_idx = start_frame;

    ROS_INFO("[RealTimeBuilder] Starting file-based playback: Frame %d to %d", start_frame, end_frame);

    while (ros::ok() && current_frame_idx <= end_frame) {
        // 3. 파일 로드 (Binary -> PCL PointCloud) 
        // FrameLoader::loadFrame은 내부적으로 클러스터 단위로 점들을 읽어 pcl::PointXYZI로 변환함 
        auto cloud = loader.loadFrame(current_frame_idx);

        if (cloud != nullptr) {
            // 4. PCL 형식을 빌더 내부 Point 형식으로 변환
            std::vector<linemapdraft_builder::data_types::Point> frame_points;
            frame_points.reserve(cloud->size());

            for (const auto& pcl_pt : *cloud) {
                linemapdraft_builder::data_types::Point p;
                p.x = pcl_pt.x;
                p.y = pcl_pt.y;
                p.z = pcl_pt.z;
                p.yaw = 0.0f; // 바이너리 구조상 yaw 정보가 없으므로 0으로 초기화 
                p.density = static_cast<uint32_t>(pcl_pt.intensity); // Intensity를 밀도로 매핑
                p.polyline_id = linemapdraft_builder::data_types::Unclassified;
                frame_points.push_back(p);
            }

            // 5. 실시간 파이프라인 실행
            // 파일 기반 재생이므로 frame_id는 "map"으로 고정하여 시각화함
            builder.processPipeline(frame_points, "map");

            ROS_INFO("Processed frame %d (%lu points)", current_frame_idx, frame_points.size());
        } else {
            // 파일이 없는 경우 에러가 아니라 건너뜀 (데이터 번호가 연속적이지 않을 수 있음)
            ROS_DEBUG("Frame %d not found, skipping...", current_frame_idx);
        }

        current_frame_idx++;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    ROS_INFO("[RealTimeBuilder] Playback complete.");
    return 0;
}