#include <ros/ros.h>
#include "toy_map_viewer/MapConverter.h"
#include "toy_map_viewer/CoordinateConverter.h"
#include "real_time_map/BatchSaver.h"

int main(int argc, char** argv) {
    // 노드 이름 초기화 (launch 파일에서 name을 덮어씌울 수 있음)
    ros::init(argc, argv, "unified_converter_node");
    
    // NodeHandle 생성 (필요하다면)
    ros::NodeHandle nh("~");

    // ROS_INFO("#############################################");
    // ROS_INFO("#        Unified Converter Node Start       #");
    // ROS_INFO("#############################################");

    // // 1. MapConverter 실행
    // {
    //     ROS_INFO("[Step 1] Running MapConverter...");
    //     MapConverter map_converter;
    //     map_converter.run();
    //     ROS_INFO("[Step 1] MapConverter Completed.");
    // }

    ROS_INFO("=============================================");
    ROS_INFO("   Real-time Polyline Generation Started     ");
    ROS_INFO("=============================================");

    // 1. 객체 초기화
    CoordinateConverter coord_converter;
    
    // 2. 처리에 필요한 프레임 인덱스 목록 가져오기 
    // (CoordinateConverter에 public 함수 getFrameIndices()가 있다고 가정)
    std::vector<int> frame_indices = coord_converter.getFrameIndices();
    int sensor_id = coord_converter.getSensorId();

    for (int idx : frame_indices) {
        // [Step 1] 좌표 변환 및 전역 맵 누적 (기존 로직)
        // processFrame을 public으로 변경하거나 wrapper 함수 필요
        if (!coord_converter.processFrame(sensor_id, idx)) {
            continue;
        }

        // 현재까지 누적된 PointCloud 가져오기 (PointXYZI 타입)
        auto global_cloud = coord_converter.getGlobalBinMap();

        // --- 실시간 Polyline 파이프라인 시작 ---

        // [Step 2] Voxel Builder (메모리 상의 PointCloud를 직접 처리)
        // 기존의 build(ctx) 대신 PointCloud를 받는 realtime 버전 함수를 만들어 호출
        std::vector<data_types::Point> voxels;
        if (!linemapdraft_builder::voxel_builder::build_realtime(global_cloud, voxels)) {
            continue;
        }

        // [Step 3] Polyline Generator (정제된 Voxel 점들로 선 생성)
        std::vector<std::vector<data_types::Point>> polylines;
        if (!linemapdraft_builder::polyline_generator::generate_realtime(voxels, polylines)) {
            continue;
        }

        // [Step 4] Polyline Simplifier (선형 단순화)
        linemapdraft_builder::polyline_simplifier::simplify_realtime(polylines);

        // [Step 5] 시각화 및 결과 발행 (RViz Marker 등)
        // publishPolylineMarkers(polylines);

        ROS_INFO("Success: Frame [%d] -> Polyline Generated.", idx);
        
        ros::spinOnce(); // ROS 콜백 처리
    }

    ROS_INFO("=============================================");
    ROS_INFO("       All Frames Processed Successfully     ");
    ROS_INFO("=============================================");

    
    // 나중에 LineMapProcessor를 만든 후 실행할 것.
    {
        ROS_INFO("[Step 3] Running BatchSaver...");
        BatchSaver batch_saver;
        batch_saver.run();
        ROS_INFO("[Step 3] BatchSaver Completed");
    }
    

    return 0;
}