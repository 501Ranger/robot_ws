#ifndef __IMU_DEVICE_HPP
#define __IMU_DEVICE_HPP

#include <fstream>
#include <sstream>
#include <vector>

#include "robot_app_pkg/sdk/device_base.hpp"
namespace robot_sdk {
class IMUDevice : public DeviceBase {
private:
    std::ifstream file_stream_;
    std::string file_path_;
    std::vector<double> current_data_;  // 存放解析后的 ax, ay, az, gx, gy, gz
public:
    IMUDevice(const int id, const std::function<void()> f) : DeviceBase("IMUDevice", id, f), file_path_("imu_data.csv") {}
    bool init() override {
        state_ = DeviceState::INITIALIZING;
        RCLCPP_INFO(rclcpp::get_logger("IMUDevice"),
                    "正在初始化设备 : %s,id : %d", name_.c_str(), id_);
        file_stream_.open(file_path_);
        if (file_stream_.is_open()) {
            state_ = DeviceState::READY;
            RCLCPP_INFO(rclcpp::get_logger("IMUDevice"),
                        "设备 : %s,id : %d初始化成功!", name_.c_str(), id_);
            return true;
        } else {
            state_ = DeviceState::ERROR;
            RCLCPP_INFO(rclcpp::get_logger("IMUDevice"),
                        "设备 : %s,id : %d初始化失败!", name_.c_str(), id_);
            return false;
        }
    }
    void start() override {
        if (state_ == DeviceState::READY) {
            state_ = DeviceState::RUNNING;
            RCLCPP_INFO(rclcpp::get_logger("IMUDevice"),
                        "设备 : %s,id : %d已启动!", name_.c_str(), id_);
        } else
            RCLCPP_WARN(rclcpp::get_logger("IMUDevice"),
                        "IMU状态未就绪，无法启动！当前状态码：%d", static_cast<int>(state_));
    }
    void stop() override {
        if (state_ == DeviceState::RUNNING) {
            state_ = DeviceState::READY;
            RCLCPP_INFO(rclcpp::get_logger("IMUDevice"), "IMU已安全停止。");
        }
    }
    void read_and_publish_data() override {
        if (state_ != DeviceState::RUNNING) return;
        std::string line;
        // 如果读到末尾，重置回开头
        if (!std::getline(file_stream_, line)) {
            file_stream_.clear();
            file_stream_.seekg(0);
            std::getline(file_stream_, line);
        }

        // 解析 CSV 行
        std::stringstream ss(line);
        std::string item;
        current_data_.clear();
        while (std::getline(ss, item, ',')) {
            try {
                current_data_.push_back(std::stod(item));
            } catch (...) {
                continue;
            }
        }

        // 模拟 IMU 采集后的业务逻辑
        if (func_) {
            func_();  // 触发回调，外部可以通过 getter 拿 current_data_
        }
    }
    std::vector<double> get_imu_data() const {
        return current_data_;
    }
    ~IMUDevice() = default;
};
}  // namespace robot_sdk
#endif
