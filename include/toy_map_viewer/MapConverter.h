#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <map>
#include <string>
#include <vector>

#include "toy_map_viewer/DataTypes.h"
#include "toy_map_viewer/LaneUtils.h"

class MapConverter {
public:
    MapConverter();
    ~MapConverter() = default;

    // 메인 실행 함수
    void run();

private:
    // 내부 유틸리티 함수
    void loadParameters();
    std::map<int, Lane> filterDenseRegion(const std::map<int, Lane>& src_map, double crop_size_m);
    
    // 경로 생성 헬퍼
    std::string getFilePath(int index);

private:
    ros::NodeHandle nh_;
    
    // 설정 변수 (LaneConfig 구조체 활용 가능)
    LaneConfig lane_config_;
    std::string package_name_;
    std::string input_folder_name_;
    std::string output_folder_name_;
    std::string base_dir_;
    std::string output_dir_;

    // 데이터 저장소
    std::map<int, Lane> global_map_;
};