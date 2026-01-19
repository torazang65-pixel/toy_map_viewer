#include "real_time_map/CoordinateConverterV1.h"

#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <fstream>
#include <algorithm>
#include <cmath>

#include "common/io.h"

namespace fs = std::filesystem;
using json = nlohmann::json;
namespace ldb = linemapdraft_builder;

struct PointXYZU {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZU,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
)

namespace {
std::vector<ldb::data_types::Point> toLdbPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::vector<ldb::data_types::Point> points;
    points.reserve(cloud->size());

    for (const auto& pt : cloud->points) {
        ldb::data_types::Point p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.yaw = pt.intensity;
        p.vz = 0.0f;
        p.polyline_id = ldb::data_types::Unclassified;
        p.density = 1;
        points.push_back(p);
    }
    return points;
}
}  // namespace

CoordinateConverterV1::CoordinateConverterV1()
    : nh_("~"),
      tf_listener_(tf_buffer_),
      global_pcd_map_(new pcl::PointCloud<pcl::PointXYZI>),
      global_bin_map_(new pcl::PointCloud<pcl::PointXYZI>) {
    if (!nh_.hasParam("file_idx")) {
        nh_.param<int>("start_index", file_idx_, 20000);
    }
    nh_.param<int>("file_idx", file_idx_, 20000);
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    nh_.param<std::string>("pred_folder", pred_folder_, "issue_laneline_pred/");

    std::string output_folder = "data/issue/converted_bin/";
    nh_.param<std::string>("output_folder", output_folder, output_folder);
    if (!output_folder.empty() && output_folder.back() != '/') {
        output_folder.push_back('/');
    }

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    base_dir_ = pkg_path + "/data/";
    sensor_dir_ = base_dir_ + "sensor/" + std::to_string(file_idx_) + "/pandar64_0/";
    output_dir_ = pkg_path + "/" + output_folder + std::to_string(file_idx_) + "/";
    pred_frames_dir_ = output_dir_ + "pred_frames/";

    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
    if (!fs::exists(pred_frames_dir_)) {
        fs::create_directories(pred_frames_dir_);
    }
}

std::string CoordinateConverterV1::getJsonPath(int frame_index) {
    return sensor_dir_ + "ego_state/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverterV1::getPcdPath(int frame_index) {
    return sensor_dir_ + "pcd/" + std::to_string(frame_index) + ".pcd";
}

std::string CoordinateConverterV1::getBinPath(int file_idx, int frame_index) {
    return base_dir_ + pred_folder_ + std::to_string(file_idx) + "/" + sensor_frame_id_ + "/" +
           std::to_string(frame_index) + ".bin";
}

bool CoordinateConverterV1::processFrame(int file_idx, int frame_index) {
    std::string json_path = getJsonPath(frame_index);
    std::string pcd_path = getPcdPath(frame_index);
    std::string bin_path = getBinPath(file_idx, frame_index);

    std::ifstream j_file(json_path);
    if (!j_file.is_open()) return false;

    json ego_data;
    try {
        j_file >> ego_data;
    } catch (...) {
        ROS_ERROR("JSON Parse Error at index %d", frame_index);
        return false;
    }

    std::string current_frame = ego_data.value("frame_id", "world");

    if (is_first_frame_) {
        target_frame_id_ = current_frame;
        is_first_frame_ = false;
        ROS_INFO("Set Target Frame to: %s", target_frame_id_.c_str());
    }

    Eigen::Matrix4f T_final;
    try {
        geometry_msgs::TransformStamped tf_s2v_msg =
            tf_buffer_.lookupTransform(vehicle_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(1.0));
        Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

        Eigen::Translation3f t_v2s(static_cast<float>(ego_data["x"]),
                                   static_cast<float>(ego_data["y"]),
                                   static_cast<float>(ego_data["z"]));
        Eigen::Quaternionf r_v2s(static_cast<float>(ego_data["q3"]),
                                 static_cast<float>(ego_data["q0"]),
                                 static_cast<float>(ego_data["q1"]),
                                 static_cast<float>(ego_data["q2"]));
        Eigen::Matrix4f T_v2s = (t_v2s * r_v2s).matrix();

        Eigen::Matrix4f T_s2t = Eigen::Matrix4f::Identity();
        if (current_frame != target_frame_id_) {
            geometry_msgs::TransformStamped tf_s2t_msg =
                tf_buffer_.lookupTransform(target_frame_id_, current_frame, ros::Time(0), ros::Duration(1.0));
            T_s2t = tf2::transformToEigen(tf_s2t_msg).matrix().cast<float>();
        }
        T_final = T_s2t * T_v2s * T_s2v;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Error at frame %d: %s", frame_index, ex.what());
        return false;
    }

    pcl::PointCloud<PointXYZU>::Ptr cloud_pcd_u(new pcl::PointCloud<PointXYZU>);
    if (pcl::io::loadPCDFile<PointXYZU>(pcd_path, *cloud_pcd_u) != -1) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto& pt_u : cloud_pcd_u->points) {
            pcl::PointXYZI pt_i;
            pt_i.x = pt_u.x;
            pt_i.y = pt_u.y;
            pt_i.z = pt_u.z;
            pt_i.intensity = static_cast<float>(pt_u.intensity) / 255.0f;
            cloud_pcd->push_back(pt_i);
        }
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_pcd, *transformed, T_final);
        *global_pcd_map_ += *transformed;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_bin(new pcl::PointCloud<pcl::PointXYZI>);
    std::ifstream ifs(bin_path, std::ios::binary);
    if (ifs.is_open()) {
        float buffer[4];
        while (ifs.read(reinterpret_cast<char*>(buffer), sizeof(float) * 4)) {
            pcl::PointXYZI pt;
            pt.x = buffer[0];
            pt.y = buffer[1];
            pt.z = buffer[2];
            pt.intensity = buffer[3];
            cloud_bin->push_back(pt);
        }
        ifs.close();
        if (!cloud_bin->empty()) {
            Eigen::Matrix3f rotation = T_final.block<3, 3>(0, 0);
            float yaw_offset = std::atan2(rotation(1, 0), rotation(0, 0));

            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_bin, *transformed, T_final);
            for (auto& pt : transformed->points) {
                pt.intensity = std::fmod(pt.intensity + yaw_offset + M_PI, 2 * M_PI) - M_PI;
            }

            *global_bin_map_ += *transformed;

            std::string frame_filename = pred_frames_dir_ + "frame_" + std::to_string(frame_index) + ".bin";
            ldb::io::write_points(frame_filename, toLdbPoints(transformed));
        }
    }

    Eigen::Vector4f vehicle_pos_in_vehicle_frame(0.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f vehicle_pos_in_target_frame = T_final * vehicle_pos_in_vehicle_frame;

    Eigen::Quaternionf q_transformed(T_final.block<3, 3>(0, 0));
    Eigen::Vector3f euler = q_transformed.toRotationMatrix().eulerAngles(0, 1, 2);

    Point6D vehicle_pose;
    vehicle_pose.x = vehicle_pos_in_target_frame(0);
    vehicle_pose.y = vehicle_pos_in_target_frame(1);
    vehicle_pose.z = vehicle_pos_in_target_frame(2);
    vehicle_pose.dx = euler(0);
    vehicle_pose.dy = euler(1);
    vehicle_pose.dz = euler(2);
    vehicle_trajectory_.push_back(vehicle_pose);

    return true;
}

void CoordinateConverterV1::saveMapToFile(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map,
                                          const std::string& filename, bool filter_mode) {
    if (map->empty()) return;

    if (filter_mode) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(map);
        voxel_filter.setLeafSize(0.3f, 0.3f, 0.3f);
        voxel_filter.filter(*filtered);
        ldb::io::write_points(filename, toLdbPoints(filtered));
        return;
    }

    ldb::io::write_points(filename, toLdbPoints(map));
}

void CoordinateConverterV1::saveVehicleTrajectory() {
    if (vehicle_trajectory_.empty()) {
        ROS_WARN("No vehicle trajectory to save");
        return;
    }

    std::vector<ldb::data_types::Point> points;
    points.reserve(vehicle_trajectory_.size());
    for (const auto& pose : vehicle_trajectory_) {
        ldb::data_types::Point p;
        p.x = static_cast<float>(pose.x);
        p.y = static_cast<float>(pose.y);
        p.z = static_cast<float>(pose.z);
        p.yaw = static_cast<float>(pose.dz);
        p.vz = 0.0f;
        p.polyline_id = 0;
        p.density = 1;
        points.push_back(p);
    }

    std::string trajectory_filename = output_dir_ + "vehicle_trajectory.bin";
    ldb::io::write_points(trajectory_filename, points);
    ROS_INFO("Saved Vehicle Trajectory to vehicle_trajectory.bin (%lu poses)", vehicle_trajectory_.size());
}

void CoordinateConverterV1::saveGlobalMaps() {
    saveMapToFile(global_pcd_map_, output_dir_ + "lidar_seq_0.bin", true);
    ROS_INFO("Saved PCD Global Map to lidar_seq_0.bin");

    saveMapToFile(global_bin_map_, output_dir_ + "lidar_seq_1.bin", false);
    ROS_INFO("Saved BIN Global Map to lidar_seq_1.bin");

    saveVehicleTrajectory();
}

void CoordinateConverterV1::run() {
    std::string ego_state_dir = sensor_dir_ + "ego_state/";
    if (!fs::exists(ego_state_dir)) {
        ROS_ERROR("Directory not found: %s", ego_state_dir.c_str());
        return;
    }

    std::vector<int> frame_indices;
    for (const auto& entry : fs::directory_iterator(ego_state_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".json") {
            try {
                frame_indices.push_back(std::stoi(entry.path().stem().string()));
            } catch (...) {}
        }
    }
    std::sort(frame_indices.begin(), frame_indices.end());

    if (frame_indices.empty()) {
        ROS_WARN("No frames found.");
        return;
    }

    ROS_INFO(">>> Starting processing %lu frames...", frame_indices.size());

    int success_count = 0;
    for (int idx : frame_indices) {
        if (processFrame(file_idx_, idx)) {
            success_count++;
            if (success_count % 50 == 0) ROS_INFO("Processed frame: %d", idx);
        }
    }

    saveGlobalMaps();
}
