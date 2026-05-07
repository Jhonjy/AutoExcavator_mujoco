// 挖掘机演示节点 - 定时航点模式
// 每隔固定时间发布一个速度指令，无位置反馈

#ifndef EXCAVATOR_DEMO__DEMO_NODE_HPP_
#define EXCAVATOR_DEMO__DEMO_NODE_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace excavator_demo {

struct Waypoint {
  double rotation;
  double boom;
  double arm;
  double bucket;
};

class DemoNode : public rclcpp::Node {
public:
  explicit DemoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void start_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr);
  void stop_callback(
      const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr);

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool running_ = false;
  int current_wp_ = 0;

  std::vector<Waypoint> waypoints_;
};

}  // namespace excavator_demo

#endif  // EXCAVATOR_DEMO__DEMO_NODE_HPP_
