# 自动挖掘节点启动文件
# 前提：excavator_bringup已经运行（硬件接口和控制器已激活）

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ==================== 启动参数 ====================
        DeclareLaunchArgument(
            "dig_velocity",
            default_value="-0.3",
            description="挖掘时动臂下降速度"
        ),
        DeclareLaunchArgument(
            "lift_velocity",
            default_value="0.3",
            description="举升速度"
        ),
        DeclareLaunchArgument(
            "swing_velocity",
            default_value="0.2",
            description="回转速度"
        ),
        DeclareLaunchArgument(
            "swing_target",
            default_value="1.57",
            description="回转目标角度(rad)，默认90度"
        ),
        DeclareLaunchArgument(
            "state_timeout",
            default_value="30.0",
            description="单个状态最大执行时间(秒)"
        ),

        # ==================== 自动挖掘节点 ====================
        Node(
            package="excavator_auto_dig",
            executable="auto_dig_node",
            name="auto_dig_node",
            output="screen",
            parameters=[{
                "dig_velocity": LaunchConfiguration("dig_velocity"),
                "lift_velocity": LaunchConfiguration("lift_velocity"),
                "swing_velocity": LaunchConfiguration("swing_velocity"),
                "swing_target": LaunchConfiguration("swing_target"),
                "state_timeout": LaunchConfiguration("state_timeout"),
            }],
        ),
    ])
