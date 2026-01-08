#include <ros/ros.h>
#include "toy_map_viewer/CoordinateConverter.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "coordinate_converter_node");
    
    CoordinateConverter converter;
    converter.run();

    return 0;
}