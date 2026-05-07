# MuJoCo可视化查看器启动文件
# 前提：excavator_bringup已经运行

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    ws_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(this_dir))))
    default_model = os.path.join(
        ws_root, "excavator_simulator_mujoco", "model", "excavator", "excavator.xml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=default_model,
            description="MuJoCo模型XML文件路径"
        ),

        Node(
            package="excavator_viewer",
            executable="mujoco_viewer",
            name="mujoco_viewer",
            output="screen",
            arguments=[LaunchConfiguration("model_path")],
        ),
    ])
