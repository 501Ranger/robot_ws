#include <chrono>
#include <std_msgs/msg/string.hpp>

#include "robot_app_pkg/sdk/device_factory.hpp"   // 对应 include/robot_app_pkg/sdk/ 下的文件
#include "robot_app_pkg/task_scheduler_base.hpp"  // 对应 include/robot_app_pkg/ 下的文件

using namespace std::chrono_literals;

class RobotCoreNode : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    TaskScheduler scheduler_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<robot_sdk::DeviceBase> chassis_;
    std::shared_ptr<robot_sdk::DeviceBase> imu_;

public:
    RobotCoreNode() : Node("robot_core_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);
        chassis_ = robot_sdk::DeviceFactory::create_device("CHASSIS", 1, [this]() {
            auto msg = std_msgs::msg::String();
            msg.data = "move:1.0";
            publisher_->publish(msg);
        });
        imu_ = robot_sdk::DeviceFactory::create_device("IMU", 2, [this]() {
            auto msg = std_msgs::msg::String();
            // msg.data = "readed";
            // publisher_->publish(msg);
            auto raw_ptr = std::dynamic_pointer_cast<robot_sdk::IMUDevice>(this->imu_);
            auto data = raw_ptr->get_imu_data();
            RCLCPP_INFO(this->get_logger(), "IMU 数据: ax=%f, ay=%f, az=%f", data[0], data[1], data[2]);
        });
        timer_ = this->create_wall_timer(1s, [this]() {
            auto task_chessis = std::make_shared<Task>(1, "ReadChassisData", [this]() { this->chassis_->read_and_publish_data(); });
            auto task_imu = std::make_shared<Task>(1, "ReadIMUData", [this]() { this->imu_->read_and_publish_data(); });
            scheduler_.submit_task(task_chessis);
            scheduler_.submit_task(task_imu);
        });
        scheduler_.start(2);
        if (chassis_->init()) {
            chassis_->start();
        }
        if (imu_->init()) {
            imu_->start();
        }
    }
    ~RobotCoreNode() {
        RCLCPP_INFO(rclcpp::get_logger("robot_core_node"),
                    "节点准备关闭，开始安全清理资源...");
        if (timer_) timer_->cancel();
        scheduler_.stop();
        if (chassis_) chassis_->stop();
        RCLCPP_INFO(rclcpp::get_logger("robot_core_node"),
                    "资源清理完毕，安全退出。");
    }
};

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotCoreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}