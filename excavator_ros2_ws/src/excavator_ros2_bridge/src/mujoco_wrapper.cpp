// MuJoCo引擎封装实现
// 负责MuJoCo模型加载、土壤插件注册、物理步进和状态读写

#include "excavator_ros2_bridge/mujoco_wrapper.hpp"

#include <cstring>
#include <iostream>

namespace excavator_ros2_bridge {

MujocoWrapper::MujocoWrapper() = default;

MujocoWrapper::~MujocoWrapper() { shutdown(); }

bool MujocoWrapper::initialize(const std::string& model_path,
                                const std::string& plugin_dir) {
  if (initialized_) {
    std::cerr << "[MujocoWrapper] 已经初始化，跳过重复初始化" << std::endl;
    return false;
  }

  // 步骤1：加载土壤插件库
  // 原始项目的main.cc通过scanPluginLibraries()加载插件
  // 这里显式指定插件目录，确保mujoco.soil插件被注册
  if (!plugin_dir.empty()) {
    std::cout << "[MujocoWrapper] 加载插件目录: " << plugin_dir << std::endl;
    mj_loadAllPluginLibraries(
        plugin_dir.c_str(),
        +[](const char* filename, int first, int count) {
          std::cout << "[MujocoWrapper] 插件库 '" << filename << "' 注册了 "
                    << count << " 个插件" << std::endl;
          for (int i = first; i < first + count; ++i) {
            std::cout << "  - " << mjp_getPluginAtSlot(i)->name << std::endl;
          }
        });
  }

  // 步骤2：加载MuJoCo模型
  char error[1024] = "";
  m_ = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m_) {
    std::cerr << "[MujocoWrapper] 模型加载失败: " << error << std::endl;
    return false;
  }
  std::cout << "[MujocoWrapper] 模型加载成功: " << model_path << std::endl;
  std::cout << "  关节数: " << m_->njnt << std::endl;
  std::cout << "  执行器数: " << m_->nu << std::endl;

  // 步骤3：创建仿真数据
  d_ = mj_makeData(m_);
  if (!d_) {
    std::cerr << "[MujocoWrapper] 仿真数据创建失败" << std::endl;
    mj_deleteModel(m_);
    m_ = nullptr;
    return false;
  }

  // 步骤4：执行一次前向运动学初始化
  mj_forward(m_, d_);

  initialized_ = true;
  return true;
}

void MujocoWrapper::step() {
  if (initialized_ && m_ && d_) {
    mj_step(m_, d_);
  }
}

double MujocoWrapper::get_joint_pos(int jnt_id) const {
  if (!m_ || !d_ || jnt_id < 0 || jnt_id >= m_->njnt) {
    return 0.0;
  }
  // qposadr存储每个关节在qpos数组中的起始索引
  return d_->qpos[m_->jnt_qposadr[jnt_id]];
}

double MujocoWrapper::get_joint_vel(int jnt_id) const {
  if (!m_ || !d_ || jnt_id < 0 || jnt_id >= m_->njnt) {
    return 0.0;
  }
  // dofadr存储每个关节在qvel数组中的起始索引
  return d_->qvel[m_->jnt_dofadr[jnt_id]];
}

void MujocoWrapper::set_actuator_ctrl(int act_id, double value) {
  if (!m_ || !d_ || act_id < 0 || act_id >= m_->nu) {
    return;
  }
  d_->ctrl[act_id] = value;
}

int MujocoWrapper::joint_name_to_id(const std::string& name) const {
  if (!m_) return -1;
  return mj_name2id(m_, mjOBJ_JOINT, name.c_str());
}

int MujocoWrapper::actuator_name_to_id(const std::string& name) const {
  if (!m_) return -1;
  return mj_name2id(m_, mjOBJ_ACTUATOR, name.c_str());
}

int MujocoWrapper::num_joints() const { return m_ ? m_->njnt : 0; }

int MujocoWrapper::num_actuators() const { return m_ ? m_->nu : 0; }

double MujocoWrapper::get_actuator_ctrl(int act_id) const {
  if (!m_ || !d_ || act_id < 0 || act_id >= m_->nu) return 0.0;
  return d_->ctrl[act_id];
}

double MujocoWrapper::get_time() const {
  if (!d_) return 0.0;
  return d_->time;
}

double MujocoWrapper::ctrl_min(int act_id) const {
  if (!m_ || act_id < 0 || act_id >= m_->nu) return 0.0;
  // actuator_ctrlrange存储[lo, hi]对
  return m_->actuator_ctrlrange[2 * act_id];
}

double MujocoWrapper::ctrl_max(int act_id) const {
  if (!m_ || act_id < 0 || act_id >= m_->nu) return 0.0;
  return m_->actuator_ctrlrange[2 * act_id + 1];
}

void MujocoWrapper::shutdown() {
  if (d_) {
    mj_deleteData(d_);
    d_ = nullptr;
  }
  if (m_) {
    mj_deleteModel(m_);
    m_ = nullptr;
  }
  initialized_ = false;
}

}  // namespace excavator_ros2_bridge
