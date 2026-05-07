// 挖掘机ROS2 Control硬件接口
// 将MuJoCo的4个velocity执行器暴露为ros2_control的关节接口
// 继承hardware_interface::SystemInterface，作为pluginlib插件动态加载

#ifndef EXCAVATOR_ROS2_BRIDGE__EXCAVATOR_HARDWARE_INTERFACE_HPP_
#define EXCAVATOR_ROS2_BRIDGE__EXCAVATOR_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "excavator_ros2_bridge/mujoco_wrapper.hpp"

namespace excavator_ros2_bridge {

// 硬件接口返回类型别名
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ExcavatorHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ExcavatorHardwareInterface)

  // ==================== 生命周期回调 ====================

  // 初始化：从hardware_info解析关节名称和数量
  CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  // 配置：实例化MujocoWrapper，加载模型，解析关节ID映射
  CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  // 激活：将所有ctrl置零，执行mj_forward初始化
  CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  // 停用：停止物理步进
  CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  // ==================== 状态/命令接口导出 ====================

  // 导出状态接口（position, velocity）
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  // 导出命令接口（velocity）
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // ==================== 实时循环回调 ====================

  // 读取：调用mj_step，从qpos/qvel读取关节状态
  hardware_interface::return_type read(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

  // 写入：将command接口的velocity值写入d->ctrl[]
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // MuJoCo引擎封装
  std::unique_ptr<MujocoWrapper> mujoco_;

  // 关节名称列表（从URDF的<ros2_control>标签解析）
  std::vector<std::string> joint_names_;

  // MuJoCo中的关节ID和执行器ID映射
  std::vector<int> mj_joint_ids_;
  std::vector<int> mj_act_ids_;

  // 状态和命令缓存
  std::vector<double> hw_positions_;   // 关节位置（从qpos读取）
  std::vector<double> hw_velocities_;  // 关节速度（从qvel读取）
  std::vector<double> hw_commands_;    // 速度指令（写入ctrl）

  // 从ros2_control参数获取的路径
  std::string model_path_;
  std::string plugin_dir_;

  // Simulate查看器模式：从外部话题读取关节状态
  bool use_sim_viewer_ = false;
  rclcpp::Node::SharedPtr sim_node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_sub_;
  std::mutex sim_mutex_;
  std::vector<double> sim_positions_;
  std::vector<double> sim_velocities_;
  bool sim_state_received_ = false;
};

}  // namespace excavator_ros2_bridge

#endif  // EXCAVATOR_ROS2_BRIDGE__EXCAVATOR_HARDWARE_INTERFACE_HPP_
