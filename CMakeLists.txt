cmake_minimum_required(VERSION 3.0.2)
project(toy_map_viewer)

add_compile_options(-std=c++17)

find_package(PCL REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib # ros::package::getPath 사용을 위해 필요
  # std_msgs 등 필요한 패키지 추가
  interactive_markers
  pcl_ros
  pcl_conversions
  nlohmann_json
  rf_tf_broadcaster
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# 실행 파일 생성
add_executable(map_converter_node 
  src/map_converter_node.cpp
  src/LaneUtils.cpp
  src/LaneCleaner.cpp
  src/LaneMerger.cpp
)
add_executable(bin_map_viewer src/bin_map_viewer.cpp)
add_executable(map_merge src/map_merge.cpp)

# 라이브러리 링크
target_link_libraries(map_converter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(bin_map_viewer ${catkin_LIBRARIES})
target_link_libraries(map_merge
  ${catkin_LIBRARIES} 
)


add_executable(map_loader_node src/map_loader.cpp)

# 의존성 연결
target_link_libraries(map_loader_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
