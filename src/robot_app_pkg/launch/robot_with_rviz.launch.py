from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    robot_node = Node(
        package="robot_app_pkg",
        executable="robot_node",
        name="robot_core_node",
        output="screen",
        parameters=[
            {"imu_source": "JY62"},
            {"imu_port": "/dev/ttyUSB0"},
            {"imu_baud": 115200},
        ],
    )

    rviz_config = os.path.join(
        get_package_share_directory("robot_app_pkg"),
        "rviz",
        "imu.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([robot_node, rviz_node])
