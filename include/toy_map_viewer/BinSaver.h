#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include "DataTypes.h"


// header-only 방식
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