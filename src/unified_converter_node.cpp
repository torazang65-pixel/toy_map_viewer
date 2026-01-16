#include <ros/ros.h>
#include "toy_map_viewer/MapConverter.h"
#include "toy_map_viewer/MapConverterV2.h"
#include "toy_map_viewer/CoordinateConverter.h"
#include "toy_map_viewer/CoordinateConverterV2.h"
#include "real_time_map/BatchSaver.h"
#include "real_time_map/LineMapProcessor.h"

int main(int argc, char** argv) {
    // 노드 이름 초기화 (launch 파일에서 name을 덮어씌울 수 있음)
    ros::init(argc, argv, "unified_converter_node");
    
    // NodeHandle 생성 (필요하다면)
    ros::NodeHandle nh("~");

    ROS_INFO("#############################################");
    ROS_INFO("#        Unified Converter Node Start       #");
    ROS_INFO("#############################################");
    
    // 1. MapConverter 실행 (v1/v2 선택)
    {
        std::string map_mode;
        nh.param<std::string>("map_converter_mode", map_mode, "v1");

        if (map_mode == "v2") {
            ROS_INFO("[Step 1] Running MapConverter (v2)...");
            MapConverterV2 map_converter;
            map_converter.run();
        } else {
            ROS_INFO("[Step 1] Running MapConverter (v1)...");
            MapConverter map_converter;
            map_converter.run();
        }
        ROS_INFO("[Step 1] MapConverter Completed.");
    }

    ROS_INFO("---------------------------------------------");
    
    // 2. CoordinateConverter 실행 (v1/v2 선택)
    {
        std::string coord_mode;
        nh.param<std::string>("coordinate_converter_mode", coord_mode, "v2");

        if (coord_mode == "v1") {
            ROS_INFO("[Step 2] Running CoordinateConverter (v1)...");
            CoordinateConverter coord_converter;
            coord_converter.run();
        } else {
            ROS_INFO("[Step 2] Running CoordinateConverter (v2)...");
            CoordinateConverterV2 coord_converter;
            coord_converter.run();
        }
        ROS_INFO("[Step 2] CoordinateConverter Completed.");
    }

    ROS_INFO("#############################################");
    ROS_INFO("#       All Conversion Tasks Finished       #");
    ROS_INFO("#############################################");
    
    // 3. BatchSaver
    {
        ROS_INFO("[Step 3] Running BatchSaver...");
        BatchSaver batch_saver;
        batch_saver.run();
        ROS_INFO("[Step 3] BatchSaver Completed");
    }
    
    // 4. LineMapProcessor
    
    {
        ROS_INFO("[Step 4] Running LineMapProcessor...");
        LineMapProcessor line_map_processor;
        line_map_processor.run();
        ROS_INFO("[Step 4] LineMapProcessor Completed");
    }

    return 0;
}
