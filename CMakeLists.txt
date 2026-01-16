cmake_minimum_required(VERSION 3.0.2)
project(toy_map_viewer)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib # ros::package::getPath 사용을 위해 필요
  sensor_msgs
  std_msgs
  geometry_msgs
  interactive_markers
  pcl_ros
  pcl_conversions
  rf_tf_broadcaster
  linemapdraft_builder
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs
)

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
)

add_library(${PROJECT_NAME}
  src/realtime_voxel_manager.cpp
  src/realtime_line_builder.cpp
  src/real_time_map/FrameLoader.cpp
)

# 노드 실행 파일명을 realtime_builder_node로 설정
add_executable(realtime_builder_node src/realtime_builder_node.cpp)
target_link_libraries(realtime_builder_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
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