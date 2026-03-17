#ifndef __DEVICE_BASE_HPP
#define __DEVICE_BASE_HPP

#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>

namespace robot_sdk {
enum class DeviceState {
    OFFLINE,
    INITIALIZING,
    RUNNING,
    READY,
    ERROR
};
class DeviceBase {
protected:
    std::string name_;
    int id_;
    DeviceState state_;
    std::function<void()> func_;

public:
    DeviceBase(const std::string& n, int id, const std::function<void()> f)
        : name_(n),
          id_(id),
          state_(DeviceState::OFFLINE),
          func_(f) {
    }
    std::string get_name() const {
        return name_;
    }
    int get_id() const {
        return id_;
    }
    DeviceState get_state() const {
        return state_;
    }
    virtual bool init() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void read_and_publish_data() = 0;
    virtual ~DeviceBase() = default;
};
}  // namespace robot_sdk
#endif