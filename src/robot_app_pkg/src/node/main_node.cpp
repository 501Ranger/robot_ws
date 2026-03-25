#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_app_pkg/sdk/device_factory.hpp"  // 对应 include/robot_app_pkg/sdk/ 下的文件
#include "robot_app_pkg/sdk/imu_device.hpp"
#include "robot_app_pkg/sdk/jy62_imu_device.hpp"
#include "robot_app_pkg/task_scheduler_base.hpp"  // 对应 include/robot_app_pkg/ 下的文件

using namespace std::chrono_literals;

// ROS2 核心节点：负责设备初始化、数据采集调度与话题发布。
class RobotCoreNode : public rclcpp::Node {
private:
    // 调试/状态字符串发布器。
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // 标准 IMU 数据发布器（加速度、角速度、姿态）。
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    // 从 IMU 姿态派生的 Pose 数据发布器。
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    // 静态 TF 广播器，用于发布 map -> imu_link。
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    // 后台任务调度器（采集任务在工作线程中执行）。
    TaskScheduler scheduler_;
    rclcpp::TimerBase::SharedPtr timer_;
    // 底盘与 IMU 设备抽象基类指针，由工厂创建具体实现。
    std::shared_ptr<robot_sdk::DeviceBase> chassis_;
    std::shared_ptr<robot_sdk::DeviceBase> imu_;

public:
    RobotCoreNode() : Node("robot_core_node") {
        // 创建各类 ROS2 发布器。
        publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("imu/pose", 10);

        // 发布静态坐标变换：此处假设 imu_link 与 map 原点重合、姿态一致。
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "imu_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;
        tf_broadcaster_->sendTransform(t);

        // 通过参数选择 IMU 类型与串口配置，便于运行时切换硬件。
        auto imu_source = this->declare_parameter<std::string>("imu_source", "JY62");
        auto imu_port = this->declare_parameter<std::string>("imu_port", "/dev/ttyUSB0");
        auto imu_baud = this->declare_parameter<int>("imu_baud", 115200);

        // 创建底盘设备；回调在设备读取后触发发布动作。
        chassis_ = robot_sdk::DeviceFactory::create_device("CHASSIS", 1, [this]() {
            auto msg = std_msgs::msg::String();
            msg.data = "move:1.0";
            publisher_->publish(msg);
        });

        // 创建 IMU 设备；统一在回调中将原始数据转换为 ROS 标准消息。
        imu_ = robot_sdk::DeviceFactory::create_imu_device(imu_source, 2, [this]() {
            auto msg = std_msgs::msg::String();
            // msg.data = "readed";
            // publisher_->publish(msg);
            auto raw_ptr = std::dynamic_pointer_cast<robot_sdk::IMUDevice>(this->imu_);
            if (raw_ptr) {
                auto data = raw_ptr->get_imu_data();
                if (data.size() >= 3) {
                    // 原始 IMUDevice 仅提供线加速度（单位 g）。
                    RCLCPP_INFO(this->get_logger(), "IMU 数据: ax=%f, ay=%f, az=%f", data[0], data[1], data[2]);
                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp = this->get_clock()->now();
                    imu_msg.header.frame_id = "imu_link";
                    // 无姿态与角速度时使用 -1 表示该字段不可用。
                    imu_msg.orientation_covariance[0] = -1.0;
                    imu_msg.angular_velocity_covariance[0] = -1.0;
                    // g -> m/s^2
                    imu_msg.linear_acceleration.x = data[0] * 9.80665;
                    imu_msg.linear_acceleration.y = data[1] * 9.80665;
                    imu_msg.linear_acceleration.z = data[2] * 9.80665;
                    imu_publisher_->publish(imu_msg);
                }
                return;
            }
            auto jy62_ptr = std::dynamic_pointer_cast<robot_sdk::JY62ImuDevice>(this->imu_);
            if (jy62_ptr) {
                auto data = jy62_ptr->get_imu_data();
                if (data.size() >= 9) {
                    // JY62 数据格式：[acc(3), gyro(3), rpy(3)]，角度单位为度。
                    RCLCPP_INFO(this->get_logger(),
                                "IMU 数据: ax=%f, ay=%f, az=%f, gx=%f, gy=%f, gz=%f, roll=%f, pitch=%f, yaw=%f",
                                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
                    sensor_msgs::msg::Imu imu_msg;
                    imu_msg.header.stamp = this->get_clock()->now();
                    imu_msg.header.frame_id = "imu_link";
                    imu_msg.orientation_covariance[0] = -1.0;
                    imu_msg.angular_velocity.x = data[3] * (M_PI / 180.0);
                    imu_msg.angular_velocity.y = data[4] * (M_PI / 180.0);
                    imu_msg.angular_velocity.z = data[5] * (M_PI / 180.0);
                    imu_msg.linear_acceleration.x = data[0] * 9.80665;
                    imu_msg.linear_acceleration.y = data[1] * 9.80665;
                    imu_msg.linear_acceleration.z = data[2] * 9.80665;
                    // 欧拉角(roll/pitch/yaw)转四元数，供下游姿态算法直接使用。
                    double roll = data[6] * (M_PI / 180.0);
                    double pitch = data[7] * (M_PI / 180.0);
                    double yaw = data[8] * (M_PI / 180.0);
                    double cr = std::cos(roll * 0.5);
                    double sr = std::sin(roll * 0.5);
                    double cp = std::cos(pitch * 0.5);
                    double sp = std::sin(pitch * 0.5);
                    double cy = std::cos(yaw * 0.5);
                    double sy = std::sin(yaw * 0.5);
                    imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
                    imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
                    imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
                    imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;
                    imu_publisher_->publish(imu_msg);

                    // 同步发布 Pose（仅包含姿态，位置保持默认值）。
                    geometry_msgs::msg::PoseStamped pose_msg;
                    pose_msg.header = imu_msg.header;
                    pose_msg.pose.orientation = imu_msg.orientation;
                    pose_publisher_->publish(pose_msg);
                }
            } }, imu_port, imu_baud);
        if (!imu_) {
            RCLCPP_ERROR(this->get_logger(), "IMU 设备创建失败，imu_source=%s", imu_source.c_str());
        }

        // 周期性提交采集任务，真正执行由调度器线程池完成。
        timer_ = this->create_wall_timer(10ms, [this]() {
            auto task_chessis = std::make_shared<Task>(1, "ReadChassisData", [this]() { this->chassis_->read_and_publish_data(); });
            auto task_imu = std::make_shared<Task>(1, "ReadIMUData", [this]() { this->imu_->read_and_publish_data(); });
            scheduler_.submit_task(task_chessis);
            // scheduler_.submit_task(task_imu);
        });

        // 启动调度器与设备。设备需先 init 成功再 start。
        scheduler_.start(2);
        if (chassis_->init()) {
            chassis_->start();
        }
        if (imu_ && imu_->init()) {
            imu_->start();
        }
    }
    ~RobotCoreNode() {
        // 按“停定时器 -> 停调度器 -> 停设备”的顺序退出，避免并发读写冲突。
        RCLCPP_INFO(rclcpp::get_logger("robot_core_node"),
                    "节点准备关闭，开始安全清理资源...");
        if (timer_) timer_->cancel();
        scheduler_.stop();
        if (chassis_) chassis_->stop();
        if (imu_) imu_->stop();
        RCLCPP_INFO(rclcpp::get_logger("robot_core_node"),
                    "资源清理完毕，安全退出。");
    }
};

int main(int argc, const char** argv) {
    // 标准 ROS2 节点生命周期：初始化 -> spin -> 关闭。
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotCoreNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
