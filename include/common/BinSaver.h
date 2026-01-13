#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "DataTypes.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Lane 데이터 저장 (기존 유지)
inline void saveToBin(const std::string& filename, const std::map<int, Lane>& map_data) {
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "파일 생성 실패: " << filename << std::endl;
        return;
    }

    uint32_t cluster_num = static_cast<uint32_t>(map_data.size());
    out.write(reinterpret_cast<const char*>(&cluster_num), sizeof(uint32_t));

    for (const auto& pair : map_data) {
        const Lane& lane = pair.second;
        
        int32_t id = lane.id;
        int32_t layer = 0;
        bool explicit_lane = lane.explicit_lane;
        uint32_t p_num = static_cast<uint32_t>(lane.points.size());

        out.write(reinterpret_cast<const char*>(&id), sizeof(int32_t));
        out.write(reinterpret_cast<const char*>(&layer), sizeof(int32_t));
        out.write(reinterpret_cast<const char*>(&explicit_lane), sizeof(bool));
        out.write(reinterpret_cast<const char*>(&p_num), sizeof(uint32_t));

        for (const auto& pt : lane.points) {
            float x = static_cast<float>(pt.x);
            float y = static_cast<float>(pt.y);
            float z = static_cast<float>(pt.z);
            
            out.write(reinterpret_cast<const char*>(&x), sizeof(float));
            out.write(reinterpret_cast<const char*>(&y), sizeof(float));
            out.write(reinterpret_cast<const char*>(&z), sizeof(float));
        }
    }
    out.close();
    std::cout << ">>> 저장 완료: " << filename << std::endl;
}

// Lidar 데이터 저장: x, y, z만 깔끔하게 저장
inline void saveLidarToBin(const std::string& filename, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "파일 생성 실패: " << filename << std::endl;
        return;
    }

    // Global Map은 1개의 거대한 클러스터로 취급
    uint32_t cluster_num = 1; 
    out.write(reinterpret_cast<const char*>(&cluster_num), sizeof(uint32_t));

    // Frame Header 작성
    int32_t id = 0; // ID는 0번으로 고정
    uint32_t p_num = static_cast<uint32_t>(cloud->size());

    out.write(reinterpret_cast<const char*>(&id), sizeof(int32_t));
    out.write(reinterpret_cast<const char*>(&p_num), sizeof(uint32_t));

    // Points 저장
    for (const auto& pt : cloud->points) {
        // PCL 포인트에서 xyz 추출 (float)
        float x = pt.x;
        float y = pt.y;
        float z = pt.z;
        float intensity = 0.0f;

        if constexpr (pcl::traits::has_intensity<pcl::PointXYZI>::value) {
            intensity = pt.intensity;
        }
        
        out.write(reinterpret_cast<const char*>(&x), sizeof(float));
        out.write(reinterpret_cast<const char*>(&y), sizeof(float));
        out.write(reinterpret_cast<const char*>(&z), sizeof(float));
        out.write(reinterpret_cast<char*>(&intensity), 4);
    }

    out.close();
    // std::cout << ">>> Global Map 저장 완료(Merged): " << filename << " (Points: " << p_num << ")" << std::endl;
}

// voxel 데이터 저장
inline void saveVoxelToBin(const std::string& filename, const std::vector<VoxelPoint>& voxels) {
    std::ofstream out(filename, std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "파일 생성 실패: " << filename << std::endl;
        return;
    }

    // Global Map처럼 Cluster Num은 1로 고정
    uint32_t cluster_num = 1; 
    out.write(reinterpret_cast<const char*>(&cluster_num), sizeof(uint32_t));

    // Frame Header (ID는 0, 포인트 개수만 저장)
    int32_t id = 0; 
    uint32_t p_num = static_cast<uint32_t>(voxels.size());

    out.write(reinterpret_cast<const char*>(&id), sizeof(int32_t));
    out.write(reinterpret_cast<const char*>(&p_num), sizeof(uint32_t));

    // Points 저장 (x, y, z, yaw, density 순서) -> 총 20바이트/점
    for (const auto& v : voxels) {
        out.write(reinterpret_cast<const char*>(&v.x), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.y), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.z), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.yaw), sizeof(float));
        out.write(reinterpret_cast<const char*>(&v.density), sizeof(uint32_t));
    }

    out.close();
    // std::cout << ">>> Voxel 저장 완료: " << filename << std::endl;
}