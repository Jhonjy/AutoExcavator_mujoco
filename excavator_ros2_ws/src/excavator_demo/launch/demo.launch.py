# 演示节点启动文件
# 前提：excavator_bringup已经运行

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="excavator_demo",
            executable="demo_node",
            name="demo_node",
            output="screen",
        ),
    ])
