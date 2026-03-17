#ifndef __CHASSIS_DEVICE_HPP
#define __CHASSIS_DEVICE_HPP

#include "robot_app_pkg/sdk/device_base.hpp"

namespace robot_sdk {
class ChassisDevice : public DeviceBase {
public:
    ChassisDevice(const int id, const std::function<void()> f) : DeviceBase("ChassisDevice", id, f) {}
    bool init() override {
        state_ = DeviceState::INITIALIZING;
        RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"),
                    "正在初始化设备 : %s,id : %d", name_.c_str(), id_);
        bool device_ready = true;
        if (device_ready) {
            state_ = DeviceState::READY;
            RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"),
                        "设备 : %s,id : %d初始化成功!", name_.c_str(), id_);
            return true;
        } else {
            state_ = DeviceState::ERROR;
            RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"),
                        "设备 : %s,id : %d初始化失败!", name_.c_str(), id_);
            return false;
        }
    }
    void start() override {
        if (state_ == DeviceState::READY) {
            state_ = DeviceState::RUNNING;
            RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"),
                        "设备 : %s,id : %d已启动!", name_.c_str(), id_);
        } else
            RCLCPP_WARN(rclcpp::get_logger("ChassisDevice"),
                        "底盘状态未就绪，无法启动！当前状态码：%d", static_cast<int>(state_));
    }
    void stop() override {
        if (state_ == DeviceState::RUNNING) {
            state_ = DeviceState::READY;
            RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"), "底盘已安全停止。");
        }
    }
    void read_and_publish_data() {
        if (state_ != DeviceState::RUNNING) return;
        try {
            if (!func_) throw std::runtime_error("空函数指针");
            func_();
            RCLCPP_INFO(rclcpp::get_logger("ChassisDevice"),
                        "底盘 [ID: %d] 成功读取并上报了一帧数据", id_);
        } catch (const std::exception& e) {
            state_ = DeviceState::ERROR;
            RCLCPP_ERROR(rclcpp::get_logger("ChassisDevice"),
                         "底盘数据读取异常：%s,设备进入 ERROR 状态", e.what());
        }
    }
    ~ChassisDevice() = default;
};
}  // namespace robot_sdk
#endif