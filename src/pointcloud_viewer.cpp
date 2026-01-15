#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <vector>
#include <map> // Publisher를 관리하기 위해 추가
#include <ros/package.h>
#include <filesystem>

namespace fs = std::filesystem;

class MapViewer {
public:
    MapViewer() : nh_("~") {
        // 1. 파라미터 로드
        nh_.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2");
        std::string pkg_path = ros::package::getPath("toy_map_viewer");
        
        bin_dir_ = pkg_path + "/data/lane_change_data_converted/Raw/" + date + "/";
        frame_id_file_ = pkg_path + "/data/lane_change_data/Raw/" + date + "/zone_info";

        std::ifstream zone_ifs(frame_id_file_);
        if (zone_ifs.is_open()) {
            zone_ifs >> map_frame_;
            ROS_INFO(">>> [%s] Detected frame_id: %s", date.c_str(), map_frame_.c_str());
            zone_ifs.close();
        } else {
            ROS_WARN(">>> [%s] zone_info not found. Defaulting to 'map'", date.c_str());
            map_frame_ = "map";
        }

        // 2. 디렉토리 내의 모든 bin 파일 처리
        loadAndPublishFiles();
    }

private:
    void loadAndPublishFiles() {
        if (!fs::exists(bin_dir_)) {
            ROS_ERROR("Directory not found: %s", bin_dir_.c_str());
            return;
        }

        for (const auto& entry : fs::directory_iterator(bin_dir_)) {
            if (entry.path().extension() == ".bin" && entry.path().stem().string().find("lidar_seq") != std::string::npos) {
                std::string file_stem = entry.path().stem().string(); // 예: lidar_seq_0
                std::string topic_name = "/global_map/" + file_stem;  // 예: /global_map/lidar_seq_0

                // 3. 파일별 전용 Publisher 생성 (없을 경우에만 advertise)
                if (file_pubs_.find(topic_name) == file_pubs_.end()) {
                    file_pubs_[topic_name] = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
                }

                publishSingleFile(entry.path().string(), topic_name);
            }
        }
    }

    void publishSingleFile(const std::string& file_path, const std::string& topic_name) {
        std::ifstream input(file_path, std::ios::binary);
        if (!input.is_open()) return;

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        float data[4];
        while (input.read(reinterpret_cast<char*>(data), sizeof(float) * 4)) {
            pcl::PointXYZI pt;
            pt.x = data[0]; pt.y = data[1]; pt.z = data[2]; pt.intensity = data[3];
            cloud->push_back(pt);
        }
        input.close();

        if (cloud->empty()) return;

        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = map_frame_;
        output.header.stamp = ros::Time::now();

        // 4. 해당 토픽으로 발행
        file_pubs_[topic_name].publish(output);
        ROS_INFO("Published %lu points to %s", cloud->size(), topic_name.c_str());
    }

    ros::NodeHandle nh_;
    std::string date, bin_dir_, frame_id_file_, map_frame_;
    // 토픽명을 키로 사용하는 Publisher 맵
    std::map<std::string, ros::Publisher> file_pubs_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_viewer_node");
    MapViewer viewer;
    ros::spin();
    return 0;
}