#include "real_time_map/CoordinateConverterV2.h"
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iostream>

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
    constexpr double kWgs84A = 6378137.0;
    constexpr double kWgs84B = 6356752.314245;
    constexpr double kWgs84A2 = kWgs84A * kWgs84A;
    constexpr double kWgs84B2 = kWgs84B * kWgs84B;
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
}

CoordinateConverterV2::CoordinateConverterV2()
    : nh_("~"),
      tf_listener_(tf_buffer_),
      global_pcd_map_(new pcl::PointCloud<pcl::PointXYZI>),
      global_bin_map_(new pcl::PointCloud<pcl::PointXYZI>) {
    nh_.param<int>("file_idx", file_idx_, 20000);
    nh_.param<std::string>("sensor_frame", sensor_frame_id_, "pandar64_0");
    nh_.param<std::string>("vehicle_frame", vehicle_frame_id_, "pcra");
    nh_.param<std::string>("pred_folder", pred_folder_, "model_output/");
    std::string output_folder = "data/issue/converted_bin/";
    nh_.param<std::string>("output_folder", output_folder, output_folder);
    if (!output_folder.empty() && output_folder.back() != '/') {
        output_folder.push_back('/');
    }

    std::string pkg_path = ros::package::getPath("realtime_line_generator");
    base_dir_ = pkg_path + "/data/";
    sensor_dir_ = base_dir_ + "sensor/" + std::to_string(file_idx_) + "/";
    sensor_frame_dir_ = sensor_dir_ + sensor_frame_id_ + "/";
    output_dir_ = pkg_path + "/" + output_folder + std::to_string(file_idx_) + "/";
    pred_frames_dir_ = output_dir_ + "pred_frames/";
    frame_id_file_ = sensor_dir_ + "zone_info";

    std::ifstream zone_ifs(frame_id_file_);
    if (zone_ifs.is_open()) {
        zone_ifs >> target_frame_id_;
        zone_ifs.close();
    } else {
        target_frame_id_ = "world";
    }

    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
    if (!fs::exists(pred_frames_dir_)) {
        fs::create_directories(pred_frames_dir_);
    }
}

geometry_msgs::Point CoordinateConverterV2::LLH2ECEF(const geometry_msgs::Point& llh) const {
    geometry_msgs::Point xyz;
    constexpr double esq = 1 - std::pow(kWgs84B / kWgs84A, 2);
    constexpr double esq_1 = 1 - esq;

    double lat = llh.x * M_PI / 180.0;
    double lon = llh.y * M_PI / 180.0;
    double alt = llh.z;

    double tmp_N = kWgs84A2 / std::sqrt(kWgs84A2 * std::pow(std::cos(lat), 2) +
                                        kWgs84B2 * std::pow(std::sin(lat), 2));

    xyz.x = (tmp_N + alt) * std::cos(lat) * std::cos(lon);
    xyz.y = (tmp_N + alt) * std::cos(lat) * std::sin(lon);
    xyz.z = (esq_1 * tmp_N + alt) * std::sin(lat);

    return xyz;
}

std::string CoordinateConverterV2::getJsonPath(int frame_index) {
    return sensor_frame_dir_ + "global_pose/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverterV2::getPcdPath(int frame_index) {
    return sensor_frame_dir_ + "pcd/" + std::to_string(frame_index) + ".pcd";
}

std::string CoordinateConverterV2::getBinPath(int file_idx, int frame_index) {
    return base_dir_ + pred_folder_ + std::to_string(file_idx) + "/" + sensor_frame_id_ + "/" +
           std::to_string(frame_index) + ".bin";
}

bool CoordinateConverterV2::processFrame(int file_idx, int frame_index) {
    std::string json_path = getJsonPath(frame_index);
    std::string pcd_path = getPcdPath(frame_index);
    std::string bin_path = getBinPath(file_idx, frame_index);

    std::ifstream j_file(json_path);
    if (!j_file.is_open()) return false;

    json global_pose;
    try {
        j_file >> global_pose;
    } catch (...) {
        ROS_ERROR("JSON Parse Error at index %d", frame_index);
        return false;
    }

    Eigen::Matrix4f T_final;
    try {
        geometry_msgs::TransformStamped tf_s2v_msg =
            tf_buffer_.lookupTransform(vehicle_frame_id_, sensor_frame_id_, ros::Time(0), ros::Duration(1.0));
        Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

        Eigen::Matrix4d T_v2w = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d T_w2z = Eigen::Matrix4d::Identity();

        geometry_msgs::Point v_llh;
        v_llh.x = global_pose["latitude"];
        v_llh.y = global_pose["longitude"];
        v_llh.z = global_pose["altitude"];
        geometry_msgs::Point ecef_xyz = LLH2ECEF(v_llh);
        T_v2w.block<3, 1>(0, 3) << ecef_xyz.x, ecef_xyz.y, ecef_xyz.z;

        double lat_rad = v_llh.x * M_PI / 180.0;
        double lon_rad = v_llh.y * M_PI / 180.0;

        tf2::Quaternion q_enu_to_ecef;
        q_enu_to_ecef.setRPY(M_PI / 2.0 - lat_rad, 0, M_PI / 2.0 + lon_rad);
        Eigen::Quaterniond R_e2e(q_enu_to_ecef.w(), q_enu_to_ecef.x(),
                                 q_enu_to_ecef.y(), q_enu_to_ecef.z());

        Eigen::Quaterniond R_v2e(static_cast<double>(global_pose["q_w"]),
                                 static_cast<double>(global_pose["q_x"]),
                                 static_cast<double>(global_pose["q_y"]),
                                 static_cast<double>(global_pose["q_z"]));

        T_v2w.block<3, 3>(0, 0) = (R_e2e * R_v2e).toRotationMatrix();

        if ("world" != target_frame_id_) {
            geometry_msgs::TransformStamped tf_w2z_msg =
                tf_buffer_.lookupTransform(target_frame_id_, "world", ros::Time(0), ros::Duration(1.0));
            T_w2z = tf2::transformToEigen(tf_w2z_msg).matrix().cast<double>();
        }

        Eigen::Matrix4f T_v2z = (T_w2z * T_v2w).cast<float>();
        T_final = T_v2z * T_s2v;
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
            float theta = buffer[3];
            while (theta < 0) theta += 2 * M_PI;
            while (theta >= M_PI) theta -= M_PI;
            pt.intensity = theta;
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

            std::string frame_filename =
                pred_frames_dir_ + "frame_" + std::to_string(frame_index) + ".bin";
            auto ldb_points = toLdbPoints(transformed);
            if (ldb::io::write_points(frame_filename, ldb_points)) {
                pred_frames_saved_++;
                pred_frame_points_saved_ += ldb_points.size();
            }
        }
    }

    Eigen::Vector4f sensor_pos_in_sensor_frame(0.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f sensor_pos_in_target_frame = T_final * sensor_pos_in_sensor_frame;

    Eigen::Quaternionf q_transformed(T_final.block<3, 3>(0, 0));
    Eigen::Vector3f euler = q_transformed.toRotationMatrix().eulerAngles(0, 1, 2);

    Point6D sensor_pose;
    sensor_pose.x = sensor_pos_in_target_frame(0);
    sensor_pose.y = sensor_pos_in_target_frame(1);
    sensor_pose.z = sensor_pos_in_target_frame(2);
    sensor_pose.dx = euler(0);
    sensor_pose.dy = euler(1);
    sensor_pose.dz = euler(2);
    vehicle_trajectory_.push_back(sensor_pose);

    return true;
}

void CoordinateConverterV2::saveMapToFile(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map,
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

void CoordinateConverterV2::saveGlobalMaps() {
    saveMapToFile(global_pcd_map_, output_dir_ + "lidar_seq_0.bin", true);
    ROS_INFO("Saved PCD Global Map to lidar_seq_0.bin");

    saveMapToFile(global_bin_map_, output_dir_ + "lidar_seq_1.bin", false);
    ROS_INFO("Saved BIN Global Map to lidar_seq_1.bin");

    saveVehicleTrajectory();
}

void CoordinateConverterV2::saveVehicleTrajectory() {
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

void CoordinateConverterV2::run() {
    std::string global_pose_dir = sensor_frame_dir_ + "global_pose/";
    if (!fs::exists(global_pose_dir)) {
        ROS_ERROR("Directory not found: %s", global_pose_dir.c_str());
        return;
    }

    std::vector<int> frame_indices;
    for (const auto& entry : fs::directory_iterator(global_pose_dir)) {
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
    ROS_INFO("Saved pred_frames: %zu files (%zu points).", pred_frames_saved_, pred_frame_points_saved_);
}
