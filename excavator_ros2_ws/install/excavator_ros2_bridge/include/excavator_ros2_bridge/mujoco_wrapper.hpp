// MuJoCo引擎封装类
// 封装MuJoCo C API，提供模型加载、物理步进、状态读取、控制写入的面向对象接口
// 用于ROS2 Control硬件接口集成

#ifndef EXCAVATOR_ROS2_BRIDGE__MUJOCO_WRAPPER_HPP_
#define EXCAVATOR_ROS2_BRIDGE__MUJOCO_WRAPPER_HPP_

#include <mujoco/mujoco.h>

#include <string>
#include <vector>

namespace excavator_ros2_bridge {

class MujocoWrapper {
public:
  MujocoWrapper();
  ~MujocoWrapper();

  // 禁止拷贝
  MujocoWrapper(const MujocoWrapper&) = delete;
  MujocoWrapper& operator=(const MujocoWrapper&) = delete;

  // 初始化：加载插件、加载模型、创建仿真数据
  // model_path: MuJoCo XML模型文件路径
  // plugin_dir: 土壤插件.so所在目录
  bool initialize(const std::string& model_path, const std::string& plugin_dir);

  // 执行一次物理步进
  void step();

  // 读取关节位置 (从qpos)
  double get_joint_pos(int jnt_id) const;

  // 读取关节速度 (从qvel)
  double get_joint_vel(int jnt_id) const;

  // 写入执行器控制指令 (写入ctrl)
  void set_actuator_ctrl(int act_id, double value);

  // 名称到ID的转换
  int joint_name_to_id(const std::string& name) const;
  int actuator_name_to_id(const std::string& name) const;

  // 获取关节数量
  int num_joints() const;

  // 获取执行器数量
  int num_actuators() const;

  // 执行器控制范围
  double ctrl_min(int act_id) const;
  double ctrl_max(int act_id) const;

  // 释放资源
  void shutdown();

  // 读取执行器控制值（调试用）
  double get_actuator_ctrl(int act_id) const;

  // 读取仿真时间（调试用）
  double get_time() const;

  // 检查是否已初始化
  bool is_initialized() const { return initialized_; }

  // 获取底层MuJoCo模型/数据指针（调试用）
  const mjModel* get_model() const { return m_; }
  const mjData* get_data() const { return d_; }

private:
  mjModel* m_ = nullptr;
  mjData* d_ = nullptr;
  bool initialized_ = false;
};

}  // namespace excavator_ros2_bridge

#endif  // EXCAVATOR_ROS2_BRIDGE__MUJOCO_WRAPPER_HPP_
