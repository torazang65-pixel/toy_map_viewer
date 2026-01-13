#pragma once

#include <ros/ros.h>
#include <deque>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "real_time_map/FrameLoader.h"

class BatchSaver {
public:
    BatchSaver();
    ~BatchSaver() = default;

    void run();

private:
    // Batch 데이터를 저장하는 함수
    void saveBatchMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, int batch_id);

private:
    ros::NodeHandle nh_;
    std::unique_ptr<FrameLoader> frame_loader_;

    // 설정 파라미터
    int start_index_;
    int batch_size_;
    int overlap_size_;
    std::string output_folder_;
    std::string output_dir_;

    // 데이터 버퍼 (Sliding Window)
    std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_buffer_;
};