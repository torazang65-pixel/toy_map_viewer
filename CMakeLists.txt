cmake_minimum_required(VERSION 3.0.2)
project(toy_map_viewer)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib # ros::package::getPath 사용을 위해 필요
  std_msgs
  interactive_markers
  pcl_ros
  pcl_conversions
  rf_tf_broadcaster
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
add_executable(bin_map_viewer
  src/bin_map_viewer.cpp
  src/viewer/PointCloudLayer.cpp
  src/viewer/LaneLayer.cpp
  src/viewer/LaneStaticLayer.cpp
  src/viewer/LidarStaticLayer.cpp
  src/viewer/VehicleTrajectoryLayer.cpp
)

add_executable(unified_converter_node
  src/unified_converter_node.cpp
  src/map_generator/MapConverter.cpp
  src/map_generator/LaneUtils.cpp
  src/map_generator/LaneCleaner.cpp
  src/map_generator/LaneMerger.cpp
  src/map_generator/CoordinateConverter.cpp
  src/real_time_map/LineMapProcessor.cpp
  src/real_time_map/FrameLoader.cpp
  src/real_time_map/BatchSaver.cpp
  src/real_time_map/VoxelBuilder.cpp
  src/real_time_map/PCALaneGenerator.cpp
  src/real_time_map/GreedyLaneGenerator.cpp
  src/real_time_map/LaneClusterer.cpp
  src/real_time_map/LanePostProcessor.cpp
)

# 라이브러리 링크
target_link_libraries(unified_converter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

target_link_libraries(bin_map_viewer 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
