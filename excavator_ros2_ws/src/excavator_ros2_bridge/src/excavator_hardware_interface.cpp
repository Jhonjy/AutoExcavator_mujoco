// 挖掘机ROS2 Control硬件接口实现
// 核心职责：在ros2_control的read/write循环中驱动MuJoCo物理引擎

#include "excavator_ros2_bridge/excavator_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace excavator_ros2_bridge {

// ==================== 生命周期回调 ====================

CallbackReturn ExcavatorHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  // 调用基类初始化，解析hardware_info中的关节定义
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // 从hardware_info获取关节数量和名称
  const auto num_joints = info.joints.size();
  joint_names_.resize(num_joints);
  for (size_t i = 0; i < num_joints; ++i) {
    joint_names_[i] = info.joints[i].name;
  }

  // 分配状态/命令缓存
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);

  // 验证每个关节都有velocity command和position/velocity state接口
  for (const auto& joint : info.joints) {
    bool has_velocity_cmd = false;
    bool has_position_state = false;
    bool has_velocity_state = false;

    for (const auto& cmd : joint.command_interfaces) {
      if (cmd.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_cmd = true;
      }
    }
    for (const auto& state : joint.state_interfaces) {
      if (state.name == hardware_interface::HW_IF_POSITION) {
        has_position_state = true;
      }
      if (state.name == hardware_interface::HW_IF_VELOCITY) {
        has_velocity_state = true;
      }
    }

    if (!has_velocity_cmd || !has_position_state || !has_velocity_state) {
      RCLCPP_FATAL(rclcpp::get_logger("ExcavatorHardwareInterface"),
                   "关节 '%s' 缺少必要的接口定义（需要velocity命令和position/velocity状态）",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
              "硬件接口初始化完成，关节数: %zu", num_joints);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExcavatorHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 从ros2_control参数获取MuJoCo模型路径和插件目录
  model_path_ = info_.hardware_parameters.count("model_path")
                    ? info_.hardware_parameters.at("model_path")
                    : "";
  plugin_dir_ = info_.hardware_parameters.count("plugin_dir")
                    ? info_.hardware_parameters.at("plugin_dir")
                    : "";

  // 是否使用Simulate查看器模式
  use_sim_viewer_ = info_.hardware_parameters.count("use_sim_viewer") &&
                    info_.hardware_parameters.at("use_sim_viewer") == "true";

  if (model_path_.empty()) {
    RCLCPP_FATAL(rclcpp::get_logger("ExcavatorHardwareInterface"),
                 "未指定model_path参数");
    return CallbackReturn::ERROR;
  }

  // 实例化并初始化MuJoCo引擎
  mujoco_ = std::make_unique<MujocoWrapper>();
  if (!mujoco_->initialize(model_path_, plugin_dir_)) {
    RCLCPP_FATAL(rclcpp::get_logger("ExcavatorHardwareInterface"),
                 "MuJoCo引擎初始化失败");
    return CallbackReturn::ERROR;
  }

  // Simulate查看器模式：创建订阅者从viewer读取关节状态
  if (use_sim_viewer_) {
    sim_node_ = std::make_shared<rclcpp::Node>("excavator_sim_bridge");
    sim_positions_.resize(joint_names_.size(), 0.0);
    sim_velocities_.resize(joint_names_.size(), 0.0);

    sim_sub_ = sim_node_->create_subscription<sensor_msgs::msg::JointState>(
        "/sim/joint_states", 10,
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(sim_mutex_);
          // 映射viewer关节名到bridge关节名
          for (size_t i = 0; i < msg->name.size(); ++i) {
            for (size_t j = 0; j < joint_names_.size(); ++j) {
              if (msg->name[i] == joint_names_[j] && i < msg->position.size()) {
                sim_positions_[j] = msg->position[i];
                if (i < msg->velocity.size()) {
                  sim_velocities_[j] = msg->velocity[i];
                }
                break;
              }
            }
          }
          sim_state_received_ = true;
        });

    // 启动spinner线程
    std::thread([this]() { rclcpp::spin(sim_node_); }).detach();

    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "Simulate查看器模式已启用，订阅 /sim/joint_states");
  }

  // 建立ROS2关节名到MuJoCo关节/执行器ID的映射
  mj_joint_ids_.resize(joint_names_.size());
  mj_act_ids_.resize(joint_names_.size());

  // MuJoCo模型中的关节和执行器名称
  // 关节: chassis, chassis piston rod, boom piston rod, arm piston rod
  // 执行器: Rotation, Boom, Arm, Bucket
  // 映射关系与URDF中定义一致
  std::vector<std::string> mj_joint_names = {
      "chassis", "chassis piston rod", "boom piston rod", "arm piston rod"};
  std::vector<std::string> mj_actuator_names = {"Rotation", "Boom", "Arm",
                                                  "Bucket"};

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    mj_joint_ids_[i] = mujoco_->joint_name_to_id(mj_joint_names[i]);
    mj_act_ids_[i] = mujoco_->actuator_name_to_id(mj_actuator_names[i]);

    if (mj_joint_ids_[i] < 0) {
      RCLCPP_FATAL(rclcpp::get_logger("ExcavatorHardwareInterface"),
                   "MuJoCo关节 '%s' 未找到", mj_joint_names[i].c_str());
      return CallbackReturn::ERROR;
    }
    if (mj_act_ids_[i] < 0) {
      RCLCPP_FATAL(rclcpp::get_logger("ExcavatorHardwareInterface"),
                   "MuJoCo执行器 '%s' 未找到", mj_actuator_names[i].c_str());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "关节映射: %s -> MuJoCo关节[%d] 执行器[%d]",
                joint_names_[i].c_str(), mj_joint_ids_[i], mj_act_ids_[i]);
  }

  // 调试：检查MuJoCo模型的equality约束和solver配置
  auto* model = mujoco_->get_model();
  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
              "MuJoCo模型信息: 关节=%d, 执行器=%d, equality约束=%d",
              model->njnt, model->nu, model->neq);
  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
              "Solver: iter=%d, tolerance=%.6f",
              model->opt.iterations, model->opt.tolerance);
  for (int i = 0; i < model->neq; ++i) {
    int type = model->eq_type[i];
    int obj1id = model->eq_obj1id[i];
    int obj2id = model->eq_obj2id[i];
    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "  eq[%d] type=%d obj1=%d obj2=%d solref=[%.6f, %.6f]",
                i, type, obj1id, obj2id,
                model->eq_solref[2*i], model->eq_solref[2*i+1]);
  }

  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
              "硬件配置完成");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ExcavatorHardwareInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 将所有控制指令置零
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = 0.0;
    mujoco_->set_actuator_ctrl(mj_act_ids_[i], 0.0);
  }

  // 读取初始关节状态
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_positions_[i] = mujoco_->get_joint_pos(mj_joint_ids_[i]);
    hw_velocities_[i] = mujoco_->get_joint_vel(mj_joint_ids_[i]);
  }

  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
              "硬件已激活，初始关节状态:");
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "  %s: pos=%.4f, vel=%.4f", joint_names_[i].c_str(),
                hw_positions_[i], hw_velocities_[i]);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ExcavatorHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // 停用时将所有控制指令置零
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_commands_[i] = 0.0;
    if (mujoco_) {
      mujoco_->set_actuator_ctrl(mj_act_ids_[i], 0.0);
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"), "硬件已停用");
  return CallbackReturn::SUCCESS;
}

// ==================== 状态/命令接口导出 ====================

std::vector<hardware_interface::StateInterface>
ExcavatorHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // 位置状态接口
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_POSITION,
                                  &hw_positions_[i]);
    // 速度状态接口
    state_interfaces.emplace_back(joint_names_[i],
                                  hardware_interface::HW_IF_VELOCITY,
                                  &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ExcavatorHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // 速度命令接口
    command_interfaces.emplace_back(joint_names_[i],
                                    hardware_interface::HW_IF_VELOCITY,
                                    &hw_commands_[i]);
  }

  return command_interfaces;
}

// ==================== 实时循环回调 ====================

hardware_interface::return_type ExcavatorHardwareInterface::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!mujoco_ || !mujoco_->is_initialized()) {
    return hardware_interface::return_type::ERROR;
  }

  // 执行一步物理仿真
  mujoco_->step();

  // 从MuJoCo读取关节状态到缓存
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    hw_positions_[i] = mujoco_->get_joint_pos(mj_joint_ids_[i]);
    hw_velocities_[i] = mujoco_->get_joint_vel(mj_joint_ids_[i]);
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ExcavatorHardwareInterface::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (!mujoco_ || !mujoco_->is_initialized()) {
    return hardware_interface::return_type::ERROR;
  }

  // 将速度指令写入MuJoCo执行器
  static int write_count = 0;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    mujoco_->set_actuator_ctrl(mj_act_ids_[i], hw_commands_[i]);
  }

  // 每2秒打印一次调试信息
  if (++write_count % 200 == 0) {
    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "write() cmd: [%.3f, %.3f, %.3f, %.3f] act_ids: [%d, %d, %d, %d]",
                hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3],
                mj_act_ids_[0], mj_act_ids_[1], mj_act_ids_[2], mj_act_ids_[3]);
    // 验证ctrl是否写入成功
    RCLCPP_INFO(rclcpp::get_logger("ExcavatorHardwareInterface"),
                "write() ctrl: [%.3f, %.3f, %.3f, %.3f]",
                mujoco_->get_actuator_ctrl(0), mujoco_->get_actuator_ctrl(1),
                mujoco_->get_actuator_ctrl(2), mujoco_->get_actuator_ctrl(3));
  }

  return hardware_interface::return_type::OK;
}

}  // namespace excavator_ros2_bridge

// 注册pluginlib插件
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    excavator_ros2_bridge::ExcavatorHardwareInterface,
    hardware_interface::SystemInterface)
