#include <ros/ros.h>
#include "toy_map_viewer/MapConverter.h"
#include "toy_map_viewer/CoordinateConverter.h"
#include "real_time_map/BatchSaver.h"

int main(int argc, char** argv) {
    // 노드 이름 초기화 (launch 파일에서 name을 덮어씌울 수 있음)
    ros::init(argc, argv, "unified_converter_node");
    
    // NodeHandle 생성 (필요하다면)
    ros::NodeHandle nh("~");

    ROS_INFO("#############################################");
    ROS_INFO("#        Unified Converter Node Start       #");
    ROS_INFO("#############################################");

    // 1. MapConverter 실행
    {
        ROS_INFO("[Step 1] Running MapConverter...");
        MapConverter map_converter;
        map_converter.run();
        ROS_INFO("[Step 1] MapConverter Completed.");
    }

    ROS_INFO("---------------------------------------------");

    // 2. CoordinateConverter 실행
    {
        ROS_INFO("[Step 2] Running CoordinateConverter...");
        CoordinateConverter coord_converter;
        coord_converter.run();
        ROS_INFO("[Step 2] CoordinateConverter Completed.");
    }

    ROS_INFO("#############################################");
    ROS_INFO("#       All Conversion Tasks Finished       #");
    ROS_INFO("#############################################");

    
    // 나중에 LineMapProcessor를 만든 후 실행할 것.
    {
        ROS_INFO("[Step 3] Running BatchSaver...");
        BatchSaver batch_saver;
        batch_saver.run();
        ROS_INFO("[Step 3] BatchSaver Completed");
    }


    
    

    return 0;
}