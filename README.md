# robot_ws

一个基于 ROS 2 的机器人应用示例工作区，当前包含 `robot_app_pkg` 包，用于演示底盘设备抽象、IMU 数据接入、任务调度、TF 发布以及 RViz 可视化。

项目重点不是完整机器人系统，而是提供一个清晰、可扩展的设备接入骨架，便于继续接串口传感器、替换数据源或扩展更多 ROS 话题。

## 功能概览

- 基于 `ament_cmake` 的 ROS 2 C++ 工程
- 统一的设备抽象基类 `DeviceBase`
- 通过 `DeviceFactory` 按类型创建底盘和 IMU 设备
- 支持两种 IMU 数据来源
- `JY62` 串口 IMU
- `CSV` 文件模拟 IMU 数据
- 内置简单任务调度器，支持后台线程执行采集任务
- 发布标准 ROS 2 消息
- `sensor_msgs/msg/Imu`
- `geometry_msgs/msg/PoseStamped`
- `std_msgs/msg/String`
- 发布静态 TF：`map -> imu_link`
- 提供 RViz 启动文件，便于直接查看 IMU 数据

## 项目结构

```text
robot_ws/
├── imu_data.csv
├── README.md
└── src/
    └── robot_app_pkg/
        ├── CMakeLists.txt
        ├── package.xml
        ├── include/robot_app_pkg/
        │   ├── task_scheduler_base.hpp
        │   └── sdk/
        │       ├── device_base.hpp
        │       ├── device_factory.hpp
        │       ├── chassis_device.hpp
        │       ├── imu_device.hpp
        │       └── jy62_imu_device.hpp
        ├── src/node/main_node.cpp
        ├── launch/robot_with_rviz.launch.py
        └── rviz/imu.rviz
```

## 核心设计

### 1. 设备抽象

项目将所有硬件设备统一抽象为 `DeviceBase`，约定以下生命周期接口：

- `init()`
- `start()`
- `stop()`
- `read_and_publish_data()`

这样做的好处是新增设备时只需要实现统一接口，不需要改动上层节点架构。

### 2. 设备工厂

`DeviceFactory` 负责根据配置创建具体设备实例：

- `CHASSIS` -> `ChassisDevice`
- `IMU` / `CSV` -> `IMUDevice`
- `JY62` -> `JY62ImuDevice`

节点层不直接依赖具体实现，便于后续替换真实硬件或增加新型号传感器。

### 3. 任务调度

`TaskScheduler` 使用线程安全队列和工作线程执行任务。当前节点中已经创建了底盘采集任务和 IMU 采集任务的结构，适合作为后续扩展多设备并发采集的基础。

### 4. ROS 节点职责

主节点 `robot_core_node` 负责：

- 初始化发布器
- 读取运行参数
- 创建并启动设备
- 定时提交采集任务
- 将原始 IMU 数据转换为标准 ROS 消息
- 发布姿态相关话题和静态 TF

## 依赖环境

建议在 Linux 环境下使用，并预先安装 ROS 2。

项目依赖如下：

- `ament_cmake`
- `rclcpp`
- `geometry_msgs`
- `sensor_msgs`
- `std_msgs`
- `tf2_ros`
- `launch_ros`
- `rviz2`
- `ament_index_python`

## 构建

在工作区根目录执行：

```bash
colcon build
source install/setup.bash
```

如果只想编译当前包：

```bash
colcon build --packages-select robot_app_pkg
source install/setup.bash
```

## 运行

### 启动节点

```bash
ros2 run robot_app_pkg robot_node
```

### 使用 launch 启动节点和 RViz

```bash
ros2 launch robot_app_pkg robot_with_rviz.launch.py
```

## 运行参数

节点当前支持以下 ROS 参数：

| 参数名 | 类型 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `imu_source` | string | `JY62` | IMU 数据源，可选 `JY62` 或 `CSV` |
| `imu_port` | string | `/dev/ttyUSB0` | JY62 串口设备路径 |
| `imu_baud` | int | `115200` | JY62 串口波特率 |

示例：

```bash
ros2 run robot_app_pkg robot_node --ros-args \
  -p imu_source:=JY62 \
  -p imu_port:=/dev/ttyUSB0 \
  -p imu_baud:=115200
```

如果使用 CSV 模拟数据：

```bash
ros2 run robot_app_pkg robot_node --ros-args -p imu_source:=CSV
```

## 发布话题

| 话题名 | 消息类型 | 说明 |
| --- | --- | --- |
| `sensor_data` | `std_msgs/msg/String` | 底盘状态或调试字符串 |
| `imu/data` | `sensor_msgs/msg/Imu` | IMU 标准数据 |
| `imu/pose` | `geometry_msgs/msg/PoseStamped` | 从 IMU 姿态派生的位姿消息 |

同时节点会发布静态坐标变换：

- `map -> imu_link`

## 支持的 IMU 数据源

### JY62 串口 IMU

`JY62ImuDevice` 会从串口读取 JY62 数据帧，并解析出：

- 线加速度 `ax, ay, az`
- 角速度 `gx, gy, gz`
- 欧拉角 `roll, pitch, yaw`

节点会进一步将欧拉角转换为四元数后发布到 `imu/data` 和 `imu/pose`。

### CSV 模拟 IMU

`IMUDevice` 会从工作区根目录下的 `imu_data.csv` 读取数据，并循环播放，适合没有实体设备时做联调或演示。

## 开发建议

如果你准备继续扩展这个项目，推荐优先从下面几项入手：

- 将更多设备接入 `DeviceFactory`
- 为不同设备拆分独立源码文件，而不是全部放在头文件中
- 为 `robot_core_node` 增加参数化话题名和坐标系配置
- 为 IMU 和底盘增加单元测试或集成测试
- 增加真实底盘控制指令和状态反馈消息
- 完善异常处理和日志分级

## 当前已知限制

- `package.xml` 中的描述和版本信息仍是占位内容，后续建议补全
- 当前代码里 IMU 采集任务提交逻辑预留了结构，但默认未启用定时提交
- `ChassisDevice` 目前更偏向示例桩实现，不是完整底盘驱动
- `CSV` 模式依赖运行目录下存在 `imu_data.csv`
- 暂未提供测试、CI 和 Docker 环境

## 许可证

本项目使用 Apache-2.0 许可证。详见 [LICENSE](./src/robot_app_pkg/LICENSE)。
