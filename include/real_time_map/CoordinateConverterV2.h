#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <cstddef>
#include <string>
#include <vector>
#include <map>
#include <Eigen/Dense>

#include <nlohmann/json.hpp>

#include <common/data_types.h>
#include "common/io.h"

namespace ldb = linemapdraft_builder;

class CoordinateConverterV2 {
public:
    CoordinateConverterV2();
    ~CoordinateConverterV2() = default;

    void run();

private:
    bool processSingleSensor(const std::string& sensor_id, int frame_idx, 
                                               std::vector<ldb::data_types::Point>& out_points);
    void saveGlobalMaps();
    void saveVehicleTrajectory();
    void saveGtFrames();
    void saveGtFramesForSequence(const std::string& gt_filename,
                                 const std::string& output_subdir);
    void saveMapToFile(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& map,
                       const std::string& filename, bool filter_mode);

    geometry_msgs::Point LLH2ECEF(const geometry_msgs::Point& llh) const;

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_pcd_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_bin_map_;

    std::vector<std::string> sensor_frame_ids;

    std::string base_dir_;
    std::string sensor_dir_;
    std::string sensor_frame_dir_;
    std::string output_dir_;
    std::string pred_frames_dir_;
    std::string pred_folder_;
    std::string vehicle_frame_id_;
    std::string target_frame_id_;
    std::string frame_id_file_;
    int file_idx_;
    double gt_roi_radius_ = 100.0;
    std::vector<linemapdraft_builder::data_types::Point6D> vehicle_trajectory_;

    std::size_t pred_frames_saved_ = 0;
    std::size_t pred_frame_points_saved_ = 0;
};
