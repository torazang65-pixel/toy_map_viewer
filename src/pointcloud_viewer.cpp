#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>
#include <vector>
#include <ros/package.h>

class MapViewer {
public:
    MapViewer() {
        ros::NodeHandle nh("~");
        
        // 1. 파라미터 로드 (실제 bin 파일 경로와 frame_id)
        nh.param<std::string>("date", date, "2025-09-26-14-21-28_maxen_v6_2");
        std::string pkg_path = ros::package::getPath("toy_map_viewer");
        bin_path_ = pkg_path + "/data/lane_change_data_converted/Raw/" + date + "/lidar_seq_0.bin";


        frame_id_file_ = pkg_path + "/data/lane_change_data/Raw/" + date.c_str() + "/zone_info";

        std::ifstream zone_ifs(frame_id_file_);
        if (zone_ifs.is_open()) {
            zone_ifs >> map_frame_; // 파일의 첫 단어(frame_id)를 읽음
            ROS_INFO(">>> [%s] Detected frame_id: %s", date.c_str(), map_frame_.c_str());
            zone_ifs.close();
        } else {
            ROS_WARN(">>> [%s] zone_info not found at %s.", 
                    date.c_str(), frame_id_file_.c_str());
        }

        pub_map_ = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 1, true);

        if (!bin_path_.empty()) {
            publishMap();
        } else {
            ROS_ERROR("Bin path is empty!");
        }
    }

    void publishMap() {
        std::ifstream input(bin_path_, std::ios::binary);
        if (!input.good()) {
            ROS_ERROR("Could not open file: %s", bin_path_.c_str());
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        int count = 0;
        
        // 2. Bin 파일 읽기 (4개 float이 한 점: x, y, z, intensity)
        while (input.good() && !input.eof()) {
            float data[4];
            input.read(reinterpret_cast<char*>(&data), sizeof(float) * 4);
            
            if (input.gcount() == sizeof(float) * 4) {
                pcl::PointXYZI pt;
                pt.x = data[0];
                pt.y = data[1];
                pt.z = data[2];
                pt.intensity = data[3];
                cloud->push_back(pt);

                if (count < 5) { // 첫 5개의 점 좌표 확인
                    ROS_INFO("Point %d: %f, %f, %f", count, pt.x, pt.y, pt.z);
                    count++;
                }
            } 
        }

        input.close();

        // 3. ROS 메시지로 변환
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = map_frame_;
        output.header.stamp = ros::Time::now();

        pub_map_.publish(output);
        ROS_INFO("Published %lu points from %s to frame %s", cloud->size(), bin_path_.c_str(), map_frame_.c_str());
    }

private:
    ros::Publisher pub_map_;
    std::string date;
    std::string bin_path_;
    std::string frame_id_file_;
    std::string map_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_viewer_node");
    MapViewer viewer;
    ros::spin();
    return 0;
}