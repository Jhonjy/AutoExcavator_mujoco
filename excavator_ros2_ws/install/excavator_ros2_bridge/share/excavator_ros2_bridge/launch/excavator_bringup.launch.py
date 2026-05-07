# 挖掘机ROS2 Control完整启动文件
# 启动顺序：robot_state_publisher → controller_manager → 控制器spawner

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # ==================== 包路径 ====================
    pkg_share = get_package_share_directory("excavator_ros2_bridge")

    # ==================== 启动参数 ====================
    # MuJoCo模型路径（指向原始项目的excavator.xml）
    default_model_path = os.path.join(
        "/home/ubuntu2204/mujoco_develop",
        "excavator_simulator_mujoco", "model", "excavator", "excavator.xml"
    )

    # 土壤插件目录
    default_plugin_dir = os.path.join(
        "/home/ubuntu2204/mujoco_develop",
        "excavator_simulator_mujoco", "build", "bin", "mujoco_plugin"
    )

    # URDF文件路径
    urdf_file = os.path.join(pkg_share, "description", "excavator.urdf.xacro")

    # 控制器配置文件路径
    controllers_file = os.path.join(pkg_share, "config", "controllers.yaml")

    # ==================== 声明启动参数 ====================
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=default_model_path,
        description="MuJoCo挖掘机模型XML文件路径"
    )

    plugin_dir_arg = DeclareLaunchArgument(
        "plugin_dir",
        default_value=default_plugin_dir,
        description="MuJoCo土壤插件目录路径"
    )

    # ==================== 获取参数值 ====================
    model_path = LaunchConfiguration("model_path")
    plugin_dir = LaunchConfiguration("plugin_dir")

    # ==================== 处理URDF ====================
    # 使用xacro处理URDF文件
    robot_description_content = Command([
        "xacro ", urdf_file,
        " model_path:=", model_path,
        " plugin_dir:=", plugin_dir,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    # ==================== 节点定义 ====================

    # 1. Robot State Publisher：发布URDF到TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 2. Controller Manager：宿主ExcavatorHardwareInterface
    #    加载controllers.yaml配置，启动硬件接口update循环
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            controllers_file,
        ],
    )

    # 3. 加载并激活joint_state_broadcaster（延迟5秒等待controller_manager就绪）
    spawn_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "controller_manager", "spawner",
                    "joint_state_broadcaster",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    # 4. 加载并激活velocity_controller（延迟6秒，在broadcaster之后）
    spawn_velocity_controller = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "controller_manager", "spawner",
                    "velocity_controller",
                    "--controller-manager", "/controller_manager",
                ],
                output="screen",
            )
        ],
    )

    # ==================== 组装启动描述 ====================
    return LaunchDescription([
        model_path_arg,
        plugin_dir_arg,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_joint_state_broadcaster,
        spawn_velocity_controller,
    ])
