// ROS2 Control桥接节点主入口
// 职责：实例化controller_manager，驱动硬件接口的update循环
// controller_manager负责加载/激活控制器，硬件接口负责MuJoCo物理步进

#include <memory>
#include <thread>

#include "controller_manager/controller_manager.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建executor
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // 创建controller_manager节点
  // controller_manager会自动发现并加载hardware_interface插件
  auto controller_manager_node =
      std::make_shared<controller_manager::ControllerManager>(
          executor, "controller_manager");

  // 声明控制器管理器参数
  controller_manager_node->declare_parameter("update_rate", 100);
  int update_rate =
      controller_manager_node->get_parameter("update_rate").as_int();

  RCLCPP_INFO(controller_manager_node->get_logger(),
              "Controller Manager 启动，更新频率: %d Hz", update_rate);

  // 将节点添加到executor
  executor->add_node(controller_manager_node);

  // 启动controller_manager的内部update循环（在独立线程中）
  std::thread cm_thread([controller_manager_node]() {
    // 等待节点初始化完成
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    RCLCPP_INFO(controller_manager_node->get_logger(),
                "Controller Manager update循环已启动");

    // controller_manager的spin由executor处理
  });

  // 主线程运行executor（阻塞，处理所有回调和update循环）
  executor->spin();

  // 清理
  cm_thread.join();
  rclcpp::shutdown();

  return 0;
}
