// 挖掘机自动作业节点实现
// 状态机：IDLE → APPROACH → DIG → LIFT → SWING → DUMP → RETURN (循环)
// 每个状态根据关节位置阈值判断是否转换到下一状态

#include "excavator_auto_dig/auto_dig_node.hpp"

#include <algorithm>
#include <cmath>

namespace excavator_auto_dig {

AutoDigNode::AutoDigNode(const rclcpp::NodeOptions& options)
    : Node("auto_dig_node", options) {
  // ==================== 声明并加载参数 ====================
  // 各状态的目标位置（raw qpos，与bridge发布的一致）
  this->declare_parameter("approach_arm_target", 0.50);   // 斗杆伸出目标
  this->declare_parameter("dig_boom_target", 0.20);       // 动臂下降深度
  this->declare_parameter("dig_duration", 4.0);           // 挖掘持续时间（秒）
  this->declare_parameter("lift_boom_target", 0.35);      // 动臂举升目标（必须高于初始0.340）
  this->declare_parameter("lift_duration", 3.0);          // 最小举升时间（秒）
  this->declare_parameter("swing_target", 1.50);          // 回转目标
  this->declare_parameter("swing_tolerance", 0.15);       // 回转容差
  this->declare_parameter("dump_bucket_target", 0.45);    // 铲斗打开目标
  this->declare_parameter("dump_duration", 3.0);          // 卸料持续时间

  // 各状态的速度（适中速度，给物理引擎时间响应）
  this->declare_parameter("dig_velocity", -0.3);
  this->declare_parameter("lift_velocity", 0.4);
  this->declare_parameter("swing_velocity", 0.4);
  this->declare_parameter("bucket_close_velocity", -0.4);
  this->declare_parameter("bucket_open_velocity", 0.4);
  this->declare_parameter("return_velocity", 0.3);

  // 超时时间（秒）
  this->declare_parameter("state_timeout", 60.0);

  // 读取参数值
  approach_arm_target_ = this->get_parameter("approach_arm_target").as_double();
  dig_boom_target_ = this->get_parameter("dig_boom_target").as_double();
  dig_duration_ = this->get_parameter("dig_duration").as_double();
  lift_boom_target_ = this->get_parameter("lift_boom_target").as_double();
  lift_duration_ = this->get_parameter("lift_duration").as_double();
  swing_target_ = this->get_parameter("swing_target").as_double();
  swing_tolerance_ = this->get_parameter("swing_tolerance").as_double();
  dump_bucket_target_ = this->get_parameter("dump_bucket_target").as_double();
  dump_duration_ = this->get_parameter("dump_duration").as_double();

  dig_velocity_ = this->get_parameter("dig_velocity").as_double();
  lift_velocity_ = this->get_parameter("lift_velocity").as_double();
  swing_velocity_ = this->get_parameter("swing_velocity").as_double();
  bucket_close_velocity_ = this->get_parameter("bucket_close_velocity").as_double();
  bucket_open_velocity_ = this->get_parameter("bucket_open_velocity").as_double();
  return_velocity_ = this->get_parameter("return_velocity").as_double();
  state_timeout_ = this->get_parameter("state_timeout").as_double();

  // ==================== 创建ROS2接口 ====================

  // 订阅关节状态
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&AutoDigNode::joint_state_callback, this, std::placeholders::_1));

  // 发布速度指令
  velocity_cmd_pub_ =
      this->create_publisher<std_msgs::msg::Float64MultiArray>(
          "/velocity_controller/commands", 10);

  // 启动服务
  start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/auto_dig/start",
      std::bind(&AutoDigNode::start_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  // 停止服务
  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/auto_dig/stop",
      std::bind(&AutoDigNode::stop_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  // 状态机定时器（50Hz）
  control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&AutoDigNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "自动挖掘节点已启动，当前状态: IDLE");
  RCLCPP_INFO(this->get_logger(), "使用 /auto_dig/start 服务启动挖掘循环");
}

// ==================== 关节状态回调 ====================

void AutoDigNode::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // 从/joint_states消息中提取各关节位置
  // 关节顺序: rotation_joint, boom_joint, arm_joint, bucket_joint
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "rotation_joint") {
      rotation_pos_ = msg->position[i];
    } else if (msg->name[i] == "boom_joint") {
      boom_pos_ = msg->position[i];
    } else if (msg->name[i] == "arm_joint") {
      arm_pos_ = msg->position[i];
    } else if (msg->name[i] == "bucket_joint") {
      bucket_pos_ = msg->position[i];
    }
  }
}

// ==================== 服务回调 ====================

void AutoDigNode::start_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (current_state_ == DigState::IDLE) {
    current_state_ = DigState::APPROACH;
    state_enter_time_ = std::chrono::steady_clock::now();
    response->success = true;
    response->message = "挖掘循环已启动，进入APPROACH状态";
    RCLCPP_INFO(this->get_logger(), "挖掘循环启动");
  } else {
    response->success = false;
    response->message = std::string("当前已在运行中，状态: ") +
                        state_name(current_state_);
    RCLCPP_WARN(this->get_logger(), "挖掘循环已在运行中");
  }
}

void AutoDigNode::stop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  current_state_ = DigState::IDLE;
  publish_zero_velocity();
  response->success = true;
  response->message = "挖掘循环已停止";
  RCLCPP_INFO(this->get_logger(), "挖掘循环停止");
}

// ==================== 状态机主循环 ====================

void AutoDigNode::control_loop() {
  if (current_state_ == DigState::IDLE) {
    return;
  }

  // 超时检查
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - state_enter_time_)
      .count();
  if (elapsed > state_timeout_) {
    RCLCPP_ERROR(this->get_logger(), "状态 %s 超时 (%.1f秒)，停止运行",
                 state_name(current_state_), elapsed);
    current_state_ = DigState::IDLE;
    publish_zero_velocity();
    return;
  }

  // 每2秒打印一次当前关节位置（调试用）
  static int loop_count = 0;
  if (++loop_count % 100 == 0) {
    RCLCPP_INFO(this->get_logger(),
                "[%s] rot=%.3f boom=%.3f arm=%.3f bucket=%.3f (elapsed=%.1fs)",
                state_name(current_state_),
                rotation_pos_, boom_pos_, arm_pos_, bucket_pos_, elapsed);
  }

  // 检查状态转换
  check_state_transition();

  // 根据当前状态发送速度指令
  switch (current_state_) {
    case DigState::APPROACH:
      // 伸出斗杆到挖掘位置
      publish_velocity(0.0, 0.0, 0.3, 0.0);
      break;

    case DigState::DIG:
      // 慢速降动臂+收铲斗挖土
      publish_velocity(0.0, dig_velocity_, 0.0, bucket_close_velocity_);
      break;

    case DigState::LIFT:
      // 举升动臂，同时收拢铲斗防止洒落
      publish_velocity(0.0, lift_velocity_, 0.0, bucket_close_velocity_);
      break;

    case DigState::SWING:
      // 回转到卸料位，保持动臂高度
      publish_velocity(swing_velocity_, 0.05, 0.0, 0.0);
      break;

    case DigState::DUMP:
      // 打开铲斗卸料
      publish_velocity(0.0, 0.0, 0.0, bucket_open_velocity_);
      break;

    case DigState::RETURN: {
      // 回转回挖掘位+恢复初始姿态
      double rot_cmd = (rotation_pos_ > 0.2) ? -swing_velocity_ : 0.0;
      double boom_cmd = (boom_pos_ < 0.30) ? lift_velocity_ : 0.0;
      double arm_cmd = (arm_pos_ > 0.50) ? -return_velocity_ : 0.0;
      publish_velocity(rot_cmd, boom_cmd, arm_cmd, 0.0);
      break;
    }

    default:
      publish_zero_velocity();
      break;
  }
}

// ==================== 状态转换检查 ====================

void AutoDigNode::check_state_transition() {
  DigState prev_state = current_state_;
  auto elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - state_enter_time_).count();

  switch (current_state_) {
    case DigState::APPROACH:
      // 斗杆伸出到位后进入挖掘
      if (arm_pos_ >= approach_arm_target_) {
        current_state_ = DigState::DIG;
      }
      break;

    case DigState::DIG:
      // 挖掘持续一段时间后举升（给铲斗时间挖入土壤）
      if (elapsed >= dig_duration_) {
        current_state_ = DigState::LIFT;
      }
      break;

    case DigState::LIFT:
      // 动臂举升到位且稳定后进入回转
      if (boom_pos_ >= lift_boom_target_ && elapsed >= lift_duration_) {
        current_state_ = DigState::SWING;
      }
      break;

    case DigState::SWING:
      // 回转到位后进入卸料
      if (std::abs(rotation_pos_ - swing_target_) < swing_tolerance_) {
        current_state_ = DigState::DUMP;
      }
      break;

    case DigState::DUMP:
      // 卸料持续一段时间后返回
      if (elapsed >= dump_duration_) {
        current_state_ = DigState::RETURN;
      }
      break;

    case DigState::RETURN:
      // 所有关节接近初始位置后循环
      if (std::abs(rotation_pos_) < 0.2 &&
          std::abs(boom_pos_ - 0.340) < 0.15 &&
          std::abs(arm_pos_ - 0.434) < 0.15) {
        current_state_ = DigState::APPROACH;
      }
      break;

    default:
      break;
  }

  // 状态发生变化时记录日志并重置计时器
  if (current_state_ != prev_state) {
    state_enter_time_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(this->get_logger(), "状态转换: %s -> %s",
                state_name(prev_state), state_name(current_state_));
  }
}

// ==================== 辅助函数 ====================

void AutoDigNode::publish_zero_velocity() {
  publish_velocity(0.0, 0.0, 0.0, 0.0);
}

void AutoDigNode::publish_velocity(double rotation, double boom,
                                     double arm, double bucket) {
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {
      clamp_velocity(rotation),
      clamp_velocity(boom),
      clamp_velocity(arm),
      clamp_velocity(bucket)};
  velocity_cmd_pub_->publish(msg);
}

double AutoDigNode::clamp_velocity(double vel) const {
  return std::max(-0.5, std::min(0.5, vel));
}

}  // namespace excavator_auto_dig

// ==================== main函数 ====================

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<excavator_auto_dig::AutoDigNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
