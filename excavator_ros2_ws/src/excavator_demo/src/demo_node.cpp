// 挖掘机演示节点 - 定时航点模式
// 每10秒发布一次速度指令，最后一个点保持

#include "excavator_demo/demo_node.hpp"

namespace excavator_demo {

DemoNode::DemoNode(const rclcpp::NodeOptions& options)
    : Node("demo_node", options) {

  waypoints_ = {
      {0.0,  0.3,  -0.1, -0.5},
      {0.0,  0.15, -0.1, -0.5},
      {0.0,  0.15,  0.0,  0.5},
      {0.0,  0.3,   0.3,  0.5},
  };

  cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/commands", 10);

  start_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/demo/start",
      std::bind(&DemoNode::start_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  stop_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/demo/stop",
      std::bind(&DemoNode::stop_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(
      std::chrono::seconds(15),
      std::bind(&DemoNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "演示节点已启动，%ld个航点", waypoints_.size());
}

void DemoNode::start_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  if (running_) {
    response->success = false;
    response->message = "已在运行中";
    return;
  }
  running_ = true;
  current_wp_ = 0;

  // 立即发送第一个航点
  const auto& wp = waypoints_[0];
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {wp.rotation, wp.boom, wp.arm, wp.bucket};
  cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "航点 1/%ld: [%.2f, %.2f, %.2f, %.2f]",
              waypoints_.size(), wp.rotation, wp.boom, wp.arm, wp.bucket);

  response->success = true;
  response->message = "演示开始";
}

void DemoNode::stop_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  running_ = false;
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {0.0, 0.0, 0.0, 0.0};
  cmd_pub_->publish(msg);
  response->success = true;
  response->message = "演示停止";
  RCLCPP_INFO(this->get_logger(), "演示停止");
}

void DemoNode::timer_callback() {
  if (!running_) return;

  // 最后一个点不再切换
  if (current_wp_ >= static_cast<int>(waypoints_.size()) - 1) return;

  current_wp_++;
  const auto& wp = waypoints_[current_wp_];
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {wp.rotation, wp.boom, wp.arm, wp.bucket};
  cmd_pub_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "航点 %d/%ld: [%.2f, %.2f, %.2f, %.2f]",
              current_wp_ + 1, waypoints_.size(),
              wp.rotation, wp.boom, wp.arm, wp.bucket);
}

}  // namespace excavator_demo

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<excavator_demo::DemoNode>());
  rclcpp::shutdown();
  return 0;
}
