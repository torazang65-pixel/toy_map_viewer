#pragma once

#include <string>

// 정적 레이어(로드 시 1회 퍼블리시) 공통 인터페이스
class StaticLayer {
public:
    explicit StaticLayer(const std::string& name) : name_(name) {}
    virtual ~StaticLayer() = default;

    virtual void loadData(const std::string& base_dir, double off_x, double off_y, double off_z) = 0;
    virtual void clear() = 0;

    std::string getName() const { return name_; }

protected:
    std::string name_;
};
