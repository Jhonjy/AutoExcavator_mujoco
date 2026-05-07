// 挖掘机自动作业节点
// 实现IDLE→APPROACH→DIG→LIFT→SWING→DUMP→RETURN的自动挖掘循环
// 通过订阅/joint_states获取关节状态，发布速度指令到/velocity_controller/commands

#ifndef EXCAVATOR_AUTO_DIG__AUTO_DIG_NODE_HPP_
#define EXCAVATOR_AUTO_DIG__AUTO_DIG_NODE_HPP_

#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace excavator_auto_dig {

// 自动挖掘状态机枚举
enum class DigState {
  IDLE,       // 等待启动指令
  APPROACH,   // 将铲斗移动到挖掘点上方
  DIG,        // 降动臂+收铲斗挖土
  LIFT,       // 举升满载铲斗
  SWING,      // 回转到卸料位
  DUMP,       // 打开铲斗卸料
  RETURN      // 回转回挖掘位，恢复初始姿态
};

// 状态名称映射（用于日志输出）
inline const char* state_name(DigState s) {
  switch (s) {
    case DigState::IDLE:     return "IDLE";
    case DigState::APPROACH: return "APPROACH";
    case DigState::DIG:      return "DIG";
    case DigState::LIFT:     return "LIFT";
    case DigState::SWING:    return "SWING";
    case DigState::DUMP:     return "DUMP";
    case DigState::RETURN:   return "RETURN";
    default:                 return "UNKNOWN";
  }
}

class AutoDigNode : public rclcpp::Node {
public:
  explicit AutoDigNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  // ==================== 回调函数 ====================

  // 关节状态订阅回调
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // 定时器回调：状态机主循环
  void control_loop();

  // 服务回调：启动挖掘循环
  void start_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  // 服务回调：停止挖掘循环
  void stop_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  // ==================== 状态转换逻辑 ====================
  void check_state_transition();

  // 发送速度指令（全零）
  void publish_zero_velocity();

  // 发送指定速度指令
  void publish_velocity(double rotation, double boom, double arm, double bucket);

  // 安全限幅
  double clamp_velocity(double vel) const;

  // ==================== ROS2接口 ====================
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ==================== 状态数据 ====================
  DigState current_state_ = DigState::IDLE;

  // 当前关节位置（从/joint_states更新）
  // 顺序: rotation_joint, boom_joint, arm_joint, bucket_joint
  double rotation_pos_ = 0.0;
  double boom_pos_ = 0.340;    // 初始值（MuJoCo ref值）
  double arm_pos_ = 0.434;
  double bucket_pos_ = 0.594;

  // 状态进入时间（用于超时保护）
  std::chrono::steady_clock::time_point state_enter_time_;

  // ==================== 可配置参数 ====================
  // 各状态的目标位置和速度（从ROS2参数加载）
  double approach_arm_target_;
  double dig_boom_target_;
  double dig_duration_;
  double lift_boom_target_;
  double lift_duration_;
  double swing_target_;
  double swing_tolerance_;
  double dump_bucket_target_;
  double dump_duration_;

  double dig_velocity_;
  double lift_velocity_;
  double swing_velocity_;
  double bucket_close_velocity_;
  double bucket_open_velocity_;
  double return_velocity_;

  // 超时时间(秒)
  double state_timeout_;
};

}  // namespace excavator_auto_dig

#endif  // EXCAVATOR_AUTO_DIG__AUTO_DIG_NODE_HPP_
