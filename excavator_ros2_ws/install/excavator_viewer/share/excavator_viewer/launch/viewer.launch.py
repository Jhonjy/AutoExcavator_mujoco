# MuJoCo可视化查看器启动文件
# 前提：excavator_bringup已经运行

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value="/home/ubuntu2204/mujoco_develop/excavator_simulator_mujoco/model/excavator/excavator.xml",
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
