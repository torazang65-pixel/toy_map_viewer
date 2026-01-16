#include <toy_map_viewer/realtime_line_builder.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace toy_map_viewer {

namespace ldb = linemapdraft_builder;

RealTimeLineBuilder::RealTimeLineBuilder(ros::NodeHandle& nh) {
    nh.param("voxel_size", voxel_size_, 0.5f);
    nh.param("yaw_voxel_num", yaw_voxel_num_, 1);

    voxel_manager_ = std::make_unique<RealTimeVoxelManager>(voxel_size_, yaw_voxel_num_);
    voxel_pub_ = nh.advertise<sensor_msgs::PointCloud2>("active_voxels", 1);
}

void RealTimeLineBuilder::callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    std::vector<ldb::data_types::Point> frame_points;
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x"), iter_y(*msg, "y"), iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        ldb::data_types::Point p;
        p.x = *iter_x; p.y = *iter_y; p.z = *iter_z;
        p.yaw = 0.0f;
        p.polyline_id = ldb::data_types::Unclassified;
        frame_points.push_back(p);
    }

    // 1단계: 보셀화 진행
    processPipeline(frame_points, msg->header.frame_id);
    frame_counter_++;
}

void RealTimeLineBuilder::processPipeline(const std::vector<ldb::data_types::Point>& frame_points, const std::string& frame_id) {
    // 1. 새로운 점들을 보셀에 누적
    voxel_manager_->accumulate(frame_points, frame_counter_);

    // 2. 오래된 보셀 제거 (Aging) <- 바로 여기서 호출!
    // 매 프레임 호출하거나, CPU 부하를 줄이기 위해 n프레임마다 호출할 수 있습니다.
    if (frame_counter_ % 10 == 0) { 
        uint32_t max_age = 100; // 예: 100프레임(약 10초) 동안 업데이트 없으면 삭제
        voxel_manager_->clearStaleVoxels(frame_counter_, max_age);
    }

    // 3. 필터링된 최신 보셀들만 가져와서 다음 단계 진행
    auto active_voxels = voxel_manager_->getFilteredPoints();
    
    // 시각화용 발행
    publishVoxelCloud(active_voxels, frame_id);

    // 2단계로 전달 (추후 구현)
    // processGeneratorStage(active_voxels);
}

void RealTimeLineBuilder::publishVoxelCloud(const std::vector<linemapdraft_builder::data_types::Point>& voxels, const std::string& frame_id) {
    if (voxels.empty()) return;

    sensor_msgs::PointCloud2 output_msg;
    output_msg.header.stamp = ros::Time::now();
    output_msg.header.frame_id = frame_id;

    // 1. PointCloud2 필드 구성 변경: "xyz" + "intensity"
    sensor_msgs::PointCloud2Modifier modifier(output_msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "intensity"); 
    modifier.resize(voxels.size());

    // 2. 각 필드에 대한 Iterator 생성
    sensor_msgs::PointCloud2Iterator<float> out_x(output_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y(output_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z(output_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> out_i(output_msg, "intensity"); // Intensity 반복자 추가

    // 3. 데이터 할당 루프
    for (const auto& v : voxels) {
        *out_x = v.x;
        *out_y = v.y;
        *out_z = v.z;
        *out_i = v.yaw; // yaw 각도 값을 intensity 필드에 저장

        ++out_x; ++out_y; ++out_z; ++out_i;
    }

    voxel_pub_.publish(output_msg);
}

} // namespace toy_map_viewer