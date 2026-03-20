#ifndef __JY62_IMU_DEVICE_HPP
#define __JY62_IMU_DEVICE_HPP

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>

#include "robot_app_pkg/sdk/device_base.hpp"

namespace robot_sdk {

class JY62ImuDevice : public DeviceBase {
private:
    std::string port_;
    int baud_;
    int fd_{-1};

    std::vector<uint8_t> rx_buffer_;
    std::vector<double> current_data_;  // ax, ay, az, gx, gy, gz
    double acc_[3]{0.0, 0.0, 0.0};
    double gyro_[3]{0.0, 0.0, 0.0};
    double rpy_[3]{0.0, 0.0, 0.0};
    bool acc_ready_{false};
    bool gyro_ready_{false};
    bool rpy_ready_{false};

    static speed_t baud_to_termios(int baud) {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            default: return B115200;
        }
    }

    static int16_t to_int16(uint8_t lo, uint8_t hi) {
        return static_cast<int16_t>((hi << 8) | lo);
    }

    static bool check_checksum(const uint8_t* frame) {
        uint8_t sum = 0;
        for (int i = 0; i < 10; ++i) sum += frame[i];
        return sum == frame[10];
    }

    void parse_frame(const uint8_t* frame) {
        // frame[0] == 0x55
        uint8_t type = frame[1];
        if (!check_checksum(frame)) return;

        if (type == 0x51) {
            int16_t ax = to_int16(frame[2], frame[3]);
            int16_t ay = to_int16(frame[4], frame[5]);
            int16_t az = to_int16(frame[6], frame[7]);
            // 单位：g (±16g)
            acc_[0] = static_cast<double>(ax) / 32768.0 * 16.0;
            acc_[1] = static_cast<double>(ay) / 32768.0 * 16.0;
            acc_[2] = static_cast<double>(az) / 32768.0 * 16.0;
            acc_ready_ = true;
        } else if (type == 0x52) {
            int16_t gx = to_int16(frame[2], frame[3]);
            int16_t gy = to_int16(frame[4], frame[5]);
            int16_t gz = to_int16(frame[6], frame[7]);
            // 单位：deg/s (±2000 dps)
            gyro_[0] = static_cast<double>(gx) / 32768.0 * 2000.0;
            gyro_[1] = static_cast<double>(gy) / 32768.0 * 2000.0;
            gyro_[2] = static_cast<double>(gz) / 32768.0 * 2000.0;
            gyro_ready_ = true;
        } else if (type == 0x53) {
            int16_t roll = to_int16(frame[2], frame[3]);
            int16_t pitch = to_int16(frame[4], frame[5]);
            int16_t yaw = to_int16(frame[6], frame[7]);
            // 单位：deg (±180)
            rpy_[0] = static_cast<double>(roll) / 32768.0 * 180.0;
            rpy_[1] = static_cast<double>(pitch) / 32768.0 * 180.0;
            rpy_[2] = static_cast<double>(yaw) / 32768.0 * 180.0;
            rpy_ready_ = true;
        }

        if (acc_ready_ && gyro_ready_ && rpy_ready_) {
            current_data_.clear();
            current_data_.push_back(acc_[0]);
            current_data_.push_back(acc_[1]);
            current_data_.push_back(acc_[2]);
            current_data_.push_back(gyro_[0]);
            current_data_.push_back(gyro_[1]);
            current_data_.push_back(gyro_[2]);
            current_data_.push_back(rpy_[0]);
            current_data_.push_back(rpy_[1]);
            current_data_.push_back(rpy_[2]);
            acc_ready_ = false;
            gyro_ready_ = false;
            rpy_ready_ = false;

            if (func_) func_();
        }
    }

public:
    JY62ImuDevice(const int id,
                  const std::function<void()> f,
                  const std::string& port,
                  int baud)
        : DeviceBase("JY62ImuDevice", id, f),
          port_(port),
          baud_(baud) {
    }

    bool init() override {
        state_ = DeviceState::INITIALIZING;
        RCLCPP_INFO(rclcpp::get_logger("JY62ImuDevice"),
                    "正在初始化设备 : %s,id : %d, port: %s, baud: %d",
                    name_.c_str(), id_, port_.c_str(), baud_);

        fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            state_ = DeviceState::ERROR;
            RCLCPP_ERROR(rclcpp::get_logger("JY62ImuDevice"),
                         "串口打开失败: %s", std::strerror(errno));
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            state_ = DeviceState::ERROR;
            RCLCPP_ERROR(rclcpp::get_logger("JY62ImuDevice"),
                         "获取串口属性失败: %s", std::strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        cfsetospeed(&tty, baud_to_termios(baud_));
        cfsetispeed(&tty, baud_to_termios(baud_));

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;  // 0.1s

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            state_ = DeviceState::ERROR;
            RCLCPP_ERROR(rclcpp::get_logger("JY62ImuDevice"),
                         "设置串口参数失败: %s", std::strerror(errno));
            ::close(fd_);
            fd_ = -1;
            return false;
        }

        state_ = DeviceState::READY;
        RCLCPP_INFO(rclcpp::get_logger("JY62ImuDevice"),
                    "设备 : %s,id : %d初始化成功!", name_.c_str(), id_);
        return true;
    }

    void start() override {
        if (state_ == DeviceState::READY) {
            state_ = DeviceState::RUNNING;
            RCLCPP_INFO(rclcpp::get_logger("JY62ImuDevice"),
                        "设备 : %s,id : %d已启动!", name_.c_str(), id_);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("JY62ImuDevice"),
                        "IMU状态未就绪，无法启动！当前状态码：%d",
                        static_cast<int>(state_));
        }
    }

    void stop() override {
        if (state_ == DeviceState::RUNNING) {
            state_ = DeviceState::READY;
            RCLCPP_INFO(rclcpp::get_logger("JY62ImuDevice"), "IMU已安全停止。");
        }
    }

    void read_and_publish_data() override {
        if (state_ != DeviceState::RUNNING || fd_ < 0) return;

        uint8_t buf[256];
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) return;

        rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);

        // 解析 11 字节帧: 0x55 + type + 8 bytes + checksum
        while (rx_buffer_.size() >= 11) {
            if (rx_buffer_[0] != 0x55) {
                rx_buffer_.erase(rx_buffer_.begin());
                continue;
            }
            if (rx_buffer_.size() < 11) break;
            parse_frame(rx_buffer_.data());
            rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 11);
        }
    }

    std::vector<double> get_imu_data() const {
        return current_data_;
    }

    ~JY62ImuDevice() override {
        if (fd_ >= 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
};

}  // namespace robot_sdk

#endif  // __JY62_IMU_DEVICE_HPP
