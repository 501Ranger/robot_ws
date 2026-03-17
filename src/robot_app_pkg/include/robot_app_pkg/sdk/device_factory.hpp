#ifndef __DEVICE_FACTORY_HPP
#define __DEVICE_FACTORY_HPP

#include <memory>
#include <string>

#include "robot_app_pkg/sdk/chassis_device.hpp"
#include "robot_app_pkg/sdk/device_base.hpp"
#include "robot_app_pkg/sdk/imu_device.hpp"

namespace robot_sdk {

class DeviceFactory {
public:
    static std::shared_ptr<DeviceBase> create_device(
        const std::string& device_type,
        int id,
        const std::function<void()>& func) {
        if (device_type == "CHASSIS") {
            auto device = std::make_shared<ChassisDevice>(id, func);
            return device;
        } else if (device_type == "IMU") {
            auto device = std::make_shared<IMUDevice>(id, func);
            return device;
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("DeviceFactory"),
                         "生产失败,类型不匹配。");
            return nullptr;
        }
    }
};

}  // namespace robot_sdk
#endif  // !__DEVICE_FACTORY_HPP