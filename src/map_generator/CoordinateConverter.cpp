#include "toy_map_viewer/CoordinateConverter.h"
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <filesystem>
#include <fstream>
#include <iostream>

#include "toy_map_viewer/BinSaver.h"

namespace fs = std::filesystem;
using json = nlohmann::json;

struct PointXYZU {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZU,
    (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)
)

CoordinateConverter::CoordinateConverter() 
    : nh_("~"), 
      tf_listener_(tf_buffer_), 
      global_pcd_map_(new pcl::PointCloud<pcl::PointXYZI>),
      global_bin_map_(new pcl::PointCloud<pcl::PointXYZI>),
      is_first_frame_(true) 
{
    // 1. 파라미터 로드
    nh_.param<int>("start_index", sensor_id_, 20000);
    nh_.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2");
    
    // 경로 설정
    std::string pkg_path = ros::package::getPath("toy_map_viewer");
    base_dir_ = pkg_path + "/data/lane_change_data/Raw/" + date + "/";
    sensor_dir_ = base_dir_ + "pandar64_1/";
    output_dir_ = pkg_path + "/data/lane_change_data_converted/Raw/" + date + "/"; // 저장 경로 변경
    frame_id_file_ = pkg_path + "/data/lane_change_data/Raw/" + date.c_str() + "/zone_info";
    bin_dir_ = pkg_path + "/data/model_output/" + date + "/pandar64_1/";
    frames_dir_ = output_dir_ + "frames/";

    std::ifstream zone_ifs(frame_id_file_);
    if (zone_ifs.is_open()) {
        zone_ifs >> target_frame_id_; // 파일의 첫 단어(frame_id)를 읽음
        ROS_INFO(">>> [%s] Detected frame_id: %s", date.c_str(), target_frame_id_.c_str());
        zone_ifs.close();
    } else {
        ROS_WARN(">>> [%s] zone_info not found at %s. Using default: 'world'", 
                date.c_str(), frame_id_file_.c_str());
    }

    // 출력 디렉토리 생성
    if (!fs::exists(output_dir_)) {
        fs::create_directories(output_dir_);
    }
}

geometry_msgs::Point
CoordinateConverter::LLH2ECEF(const geometry_msgs::Point &llh) const {
  geometry_msgs::Point xyz;

  constexpr double esq = 1 - std::pow(CONST_b / CONST_a, 2);
  constexpr double esq_1 = 1 - esq;
  double lat, lon, alt;
  double tmp_N;

  lat = llh.x * M_PI / 180.0;
  lon = llh.y * M_PI / 180.0;
  alt = llh.z;

  // LLH -> ECEF
  tmp_N = CONST_a2 / sqrt(CONST_a2 * pow(cos(lat), 2) +
                                 CONST_b2 * pow(sin(lat), 2));
  xyz.x = (tmp_N + alt) * cos(lat) * cos(lon);
  xyz.y = (tmp_N + alt) * cos(lat) * sin(lon);
  xyz.z = (esq_1 * tmp_N + alt) * sin(lat);

  return xyz;
}

std::string CoordinateConverter::getJsonPath(int frame_index) {
    return sensor_dir_ + "global_pose/" + std::to_string(frame_index) + ".json";
}

std::string CoordinateConverter::getPcdPath(int frame_index) {
    return sensor_dir_ + "pcd/" + std::to_string(frame_index) + ".pcd";
}

std::string CoordinateConverter::getBinPath(int frame_index) {
    return bin_dir_ + std::to_string(frame_index) + ".bin";
}



bool CoordinateConverter::processFrame(int sensor_id, int frame_index) {
    std::string json_path = getJsonPath(frame_index);
    std::string pcd_path = getPcdPath(frame_index);
    std::string bin_path = getBinPath(frame_index);

    // 1. JSON 로드
    std::ifstream j_file(json_path);
    if (!j_file.is_open()) return false;
    
    json global_pose;
    try {
        j_file >> global_pose;
    } catch (...) {
        ROS_ERROR("JSON Parse Error at index %d", frame_index);
        return false;
    }

    // =========================================================
    // [핵심] Map Merge 스타일의 고속 행렬 변환 로직
    // =========================================================
    // 2. 변환 행렬(T_final) 계산
    Eigen::Matrix4f T_final;
    try {
        // [Step 1] Sensor to Vehicle (항상 작으므로 float 가능)
        geometry_msgs::TransformStamped tf_s2v_msg = 
            tf_buffer_.lookupTransform("pcra", "pandar64_0", ros::Time(0), ros::Duration(1.0));
        Eigen::Matrix4f T_s2v = tf2::transformToEigen(tf_s2v_msg).matrix().cast<float>();

        // [Step 2] 고정밀도(double) 연산을 위한 Matrix4d 사용 (ECEF 오차 방지)
        Eigen::Matrix4d T_v2w = Eigen::Matrix4d::Identity(); // Vehicle to World(ECEF)
        Eigen::Matrix4d T_w2z = Eigen::Matrix4d::Identity(); // World to Zone

        // 2-1. 차량의 위치(Translation)를 ECEF로 변환
        geometry_msgs::Point v_llh;
        v_llh.x = global_pose["latitude"];
        v_llh.y = global_pose["longitude"];
        v_llh.z = global_pose["altitude"];
        geometry_msgs::Point ecef_xyz = LLH2ECEF(v_llh); // MapTFBroadcaster와 동일 로직

        T_v2w.block<3, 1>(0, 3) << ecef_xyz.x, ecef_xyz.y, ecef_xyz.z;

        // 2-2. 차량의 자세(Rotation) 계산: ENU to ECEF 기준 적용
        double lat_rad = v_llh.x * M_PI / 180.0;
        double lon_rad = v_llh.y * M_PI / 180.0;

        // 해당 위경도에서의 지평면(ENU) 회전 행렬
        tf2::Quaternion q_enu_to_ecef;
        q_enu_to_ecef.setRPY(M_PI / 2.0 - lat_rad, 0, M_PI / 2.0 + lon_rad);
        Eigen::Quaterniond R_e2e(q_enu_to_ecef.w(), q_enu_to_ecef.x(), q_enu_to_ecef.y(), q_enu_to_ecef.z());
        
        // JSON에 포함된 차량의 자세 (Vehicle to ENU)
        Eigen::Quaterniond R_v2e(static_cast<double>(global_pose["q_w"]), 
                                 static_cast<double>(global_pose["q_x"]), 
                                 static_cast<double>(global_pose["q_y"]), 
                                 static_cast<double>(global_pose["q_z"]));

        // 최종 회전 결합: R_ecef = R_enu_to_ecef * R_veh_to_enu
        T_v2w.block<3, 3>(0, 0) = (R_e2e * R_v2e).toRotationMatrix();

        // 2-3. World(ECEF) to Target Zone 변환 조회
        if ("world" != target_frame_id_) {
            geometry_msgs::TransformStamped tf_w2z_msg = 
                tf_buffer_.lookupTransform(target_frame_id_, "world", ros::Time(0), ros::Duration(1.0));
            T_w2z = tf2::transformToEigen(tf_w2z_msg).matrix().cast<double>();
        }

        // [Step 3] 글로벌 상쇄 연산 (Double 환경에서 큰 수치를 먼저 제거)
        // T_v2z 결과값은 수십~수백 미터 단위로 작아지므로 float 캐스팅이 안전함
        Eigen::Matrix4f T_v2z = (T_w2z * T_v2w).cast<float>();
        
        // 최종 행렬: Zone <- Vehicle <- Sensor
        T_final = T_v2z * T_s2v;

        // 샘플 포인트 로그 (정상 범위인지 확인용)
        Eigen::Vector4f test_pt(0, 0, 0, 1);
        Eigen::Vector4f transformed_pt = T_final * test_pt;
        ROS_INFO("Frame %d Target Pos: x=%.2f, y=%.2f, z=%.2f", 
                 frame_index, transformed_pt.x(), transformed_pt.y(), transformed_pt.z());

        // 차량의 "하늘 방향(Up-vector)"이 최종 좌표계(Zone)에서 어디를 향하는지 확인
        Eigen::Vector3f vehicle_up(0, 0, 1); // 차량 기준 위쪽
        Eigen::Matrix3f R_final = T_final.block<3, 3>(0, 0);
        Eigen::Vector3f map_up = R_final * vehicle_up;

        //ROS_INFO("Map Up-Vector: x=%.2f, y=%.2f, z=%.2f", map_up.x(), map_up.y(), map_up.z());

    } catch (tf2::TransformException &ex) {
        ROS_WARN("TF Error at frame %d: %s", frame_index, ex.what());
        return false;
    }

    // 3. PCD 데이터 로드 및 변환
    pcl::PointCloud<PointXYZU>::Ptr cloud_pcd_u(new pcl::PointCloud<PointXYZU>);
    if (pcl::io::loadPCDFile<PointXYZU>(pcd_path, *cloud_pcd_u) != -1) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_pcd(new pcl::PointCloud<pcl::PointXYZI>);

        for (const auto& pt_u : cloud_pcd_u->points) {
            pcl::PointXYZI pt_i;
            pt_i.x = pt_u.x;
            pt_i.y = pt_u.y;
            pt_i.z = pt_u.z;

            pt_i.intensity = static_cast<float>(pt_u.intensity)/255.0f;

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
            pt.x = buffer[0]; pt.y = buffer[1]; pt.z = buffer[2];
            float theta = buffer[3];
            // theta 정규화
            while (theta < 0) theta += 2 * M_PI;
            while (theta >= M_PI) theta -= M_PI;
            pt.intensity = theta;
            cloud_bin->push_back(pt);
        }
        ifs.close();
        if (!cloud_bin->empty()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::transformPointCloud(*cloud_bin, *transformed, T_final);
            *global_bin_map_ += *transformed;

            std::string frame_filename = frames_dir_ + "frame_" + std::to_string(frame_index) + ".bin";
            saveLidarToBin(frame_filename, transformed);
        }
    }

    return true;
}

void CoordinateConverter::saveMapToFile(const pcl::PointCloud<pcl::PointXYZI>::Ptr& map, const std::string& filename, bool filter_mode) {
    if (map->empty()) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr to_save = map;

    if (filter_mode) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::ApproximateVoxelGrid<pcl::PointXYZI> voxel_filter;
        voxel_filter.setInputCloud(map);
        voxel_filter.setLeafSize(0.1f, 0.1f, 0.1f); // 더 촘촘하게 필터링
        voxel_filter.filter(*filtered);
        to_save = filtered; // 필터링된 데이터를 저장 대상으로 변경
    }

    // BinSaver를 쓰지 않고 직접 안전하게 저장하는 법 (형식 강제)
    std::ofstream fout(filename, std::ios::binary);
    for (const auto& pt : to_save->points) {
        float data[4] = {pt.x, pt.y, pt.z, pt.intensity}; // X, Y, Z, I 순서 강제
        fout.write(reinterpret_cast<char*>(data), sizeof(data));
    }
    fout.close();
}

void CoordinateConverter::saveGlobalMaps() {
    // PCD 결과 저장
    saveMapToFile(global_pcd_map_, output_dir_ + "lidar_seq_0.bin", false);
    ROS_INFO("Saved PCD Global Map to lidar_seq_0.bin");

    // BIN 결과 저장
    saveMapToFile(global_bin_map_, output_dir_ + "lidar_seq_1.bin", false);
    ROS_INFO("Saved BIN Global Map to lidar_seq_1.bin");

}

void CoordinateConverter::run() {
    // 1. 파일 검색
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

    // 2. 순차 처리
    int success_count = 0;
    for (int idx : frame_indices) {
        if (processFrame(sensor_id_, idx)) {
            success_count++;
            if (success_count % 50 == 0) ROS_INFO("Processed frame: %d", idx);
        }
    }

    // 3. 결과 저장
    saveGlobalMaps();
}