cmake_minimum_required(VERSION 3.0.2)
project(realtime_line_generator)

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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp roslib sensor_msgs geometry_msgs interactive_markers pcl_ros pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
add_library(${PROJECT_NAME}
  src/gt_generator/MapConverterV1.cpp
  src/gt_generator/MapConverterV2.cpp
  src/real_time_map/CoordinateConverterV1.cpp
  src/real_time_map/CoordinateConverterV2.cpp
  src/gt_generator/LaneUtils.cpp
  src/realtime_line_generator/realtime_voxel_manager.cpp
  src/realtime_line_generator/realtime_line_builder.cpp
  src/real_time_map/FrameLoader.cpp
  src/realtime_line_generator/polyline_generator.cpp
  src/realtime_line_generator/polyline_simplifier.cpp
  src/realtime_line_generator/post_processor.cpp
  src/viewer/viewer_animation_loader.cpp
  src/viewer/viewer_static_map_loader.cpp
  src/common/io.cpp               
  src/common/data_types.cpp       
  src/evaluator/polyline_evaluator.cpp
  src/evaluator/polyline_evaluator_coverage.cpp
  src/evaluator/realtime_evaluator.cpp
  src/gt_generator/LaneCleaner.cpp
  src/gt_generator/LaneMerger.cpp
  src/gt_generator/LaneUtils.cpp
)

# 노드 실행 파일명을 realtime_builder_node로 설정
add_executable(realtime_builder_node src/nodes/realtime_builder_node.cpp)
target_link_libraries(realtime_builder_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(realtime_viewer_node src/nodes/realtime_viewer_node.cpp)
target_link_libraries(realtime_viewer_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
