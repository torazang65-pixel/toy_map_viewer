#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <common/data_types.h>

class CoordinateConverterV1 {
public:
    CoordinateConverterV1();
    ~CoordinateConverterV1() = default;

    void run();

private:
    bool processFrame(int file_idx, int frame_index);
    void saveGlobalMaps();
    void saveMapToFile(const typename pcl::PointCloud<pcl::PointXYZI>::Ptr& map,
                       const std::string& filename, bool filter_mode);
    void saveVehicleTrajectory();

    std::string getJsonPath(int frame_index);
    std::string getPcdPath(int frame_index);
    std::string getBinPath(int file_idx, int frame_index);

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr global_pcd_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_bin_map_;

    std::string base_dir_;
    std::string sensor_dir_;
    std::string output_dir_;
    std::string pred_frames_dir_;
    std::string pred_folder_;
    std::string sensor_frame_id_;
    std::string vehicle_frame_id_;
    std::string target_frame_id_;
    int file_idx_ = 0;
    bool is_first_frame_ = true;
    std::vector<linemapdraft_builder::data_types::Point6D> vehicle_trajectory_;
};
