// MuJoCo Simulate查看器 + ROS2集成
// 基于原始 excavator_simulator/main.cc，添加ROS2关节控制和状态发布
// 保留完整的Simulate UI（左侧选项面板、控制滑块等）
//
// 功能：
//   1. 订阅 /velocity_controller/commands (std_msgs/Float64MultiArray)
//      设置4个执行器的velocity控制值
//   2. 发布 /joint_states (sensor_msgs/JointState)
//      发布所有关节的位置和速度
//   3. 保留原始Simulate UI的所有功能（选项面板、控制滑块、鼠标交互等）
//
// 用法：
//   ros2 run excavator_simulate_ros2 simulate_ros2 [model.xml]

#include <mujoco/mujoco.h>
#include <glfw_adapter.h>
#include <simulate.h>
#include <array_safety.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

// ROS2头文件
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"


namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

//--------------------------------- constants ----------------------------------
const double syncMisalign = 0.1;
const double simRefreshFraction = 0.7;
const int kErrorLength = 1024;

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// control noise variables
mjtNum* ctrlnoise = nullptr;

using Seconds = std::chrono::duration<double>;

// ==================== ROS2全局变量 ====================

// ROS2节点
rclcpp::Node::SharedPtr ros_node = nullptr;

// 执行器控制值缓存（由ROS2回调写入，由物理循环读取）
// 顺序: Rotation, Boom, Arm, Bucket
double cmd_buffer[4] = {0.0, 0.0, 0.0, 0.0};
std::mutex cmd_mutex;
bool cmd_received = false;

// MuJoCo关节名称（用于发布joint_states）
std::vector<std::string> mj_joint_names;
std::vector<int> mj_joint_ids;

// ==================== ROS2回调 ====================

void velocity_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() >= 4) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    for (int i = 0; i < 4; ++i) {
      cmd_buffer[i] = std::clamp(msg->data[i], -0.5, 0.5);
    }
    cmd_received = true;
  }
}

// 发布关节状态
// MuJoCo关节名到ROS2关节名的映射
std::string mj_to_ros_name(const std::string& mj_name) {
  if (mj_name == "chassis") return "rotation_joint";
  if (mj_name == "chassis piston rod") return "boom_joint";
  if (mj_name == "boom piston rod") return "arm_joint";
  if (mj_name == "arm piston rod") return "bucket_joint";
  return mj_name;  // 其他关节保留原名
}

void publish_joint_states() {
  if (!ros_node || !m || !d) return;

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = ros_node->now();

  for (size_t i = 0; i < mj_joint_names.size(); ++i) {
    int jnt_id = mj_joint_ids[i];
    if (jnt_id >= 0 && jnt_id < m->njnt) {
      msg.name.push_back(mj_to_ros_name(mj_joint_names[i]));
      msg.position.push_back(d->qpos[m->jnt_qposadr[jnt_id]]);
      msg.velocity.push_back(d->qvel[m->jnt_dofadr[jnt_id]]);
    }
  }

  // 静态publisher（只创建一次）
  // 使用/sim/joint_states避免和bridge的/joint_states冲突
  static auto pub = ros_node->create_publisher<sensor_msgs::msg::JointState>(
      "/sim/joint_states", 10);
  pub->publish(msg);
}

//------------------------------ plugin handling -------------------------------

void scanPluginLibraries() {
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // 直接使用原始项目的插件目录
  static std::string plugin_dir_str =
      std::string(EXCAVATOR_WS_ROOT) +
      "/excavator_simulator_mujoco/build/bin/mujoco_plugin";
  const char* plugin_dir = plugin_dir_str.c_str();

  std::printf("Loading plugins from: %s\n", plugin_dir);
  mj_loadAllPluginLibraries(
      plugin_dir, +[](const char* filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//--------------------------------- simulation ---------------------------------

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  if (!filename[0]) return nullptr;

  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  if (mju::strlen_arr(filename) > 4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) mju::strcpy_arr(loadError, "could not load binary model");
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') loadError[error_length-1] = '\0';
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  if (loadError[0]) {
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  return mnew;
}

// 初始化ROS2关节名称映射（在模型加载后调用）
void init_joint_mapping() {
  mj_joint_names.clear();
  mj_joint_ids.clear();

  // 发布所有MuJoCo关节的状态
  for (int i = 0; i < m->njnt; ++i) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, i);
    if (name) {
      mj_joint_names.push_back(std::string(name));
      mj_joint_ids.push_back(i);
    }
  }

  std::printf("[ROS2] 关节映射已更新，共 %zu 个关节\n", mj_joint_names.size());
}

// 物理循环
void PhysicsLoop(mj::Simulate& sim) {
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  int terrain_id;
  int bucket_soil_1_id;
  int bucket_soil_2_id;
  int soil_id;
  bool soil_plugin;

  // 获取执行器ID
  int act_ids[4] = {-1, -1, -1, -1};
  static const char* act_names[4] = {"Rotation", "Boom", "Arm", "Bucket"};

  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);
        mj_deleteData(d);
        mj_deleteModel(m);
        m = mnew;
        d = dnew;
        mj_forward(m, d);

        free(ctrlnoise);
        ctrlnoise = (mjtNum*) malloc(sizeof(mjtNum)*m->nu);
        mju_zero(ctrlnoise, m->nu);

        // 更新关节映射和执行器ID
        init_joint_mapping();
        for (int i = 0; i < 4; ++i) {
          act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, act_names[i]);
        }
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);
        mj_deleteData(d);
        mj_deleteModel(m);
        m = mnew;
        d = dnew;
        mj_forward(m, d);

        free(ctrlnoise);
        ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
        mju_zero(ctrlnoise, m->nu);

        init_joint_mapping();
        for (int i = 0; i < 4; ++i) {
          act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, act_names[i]);
        }
      }
    }

    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    terrain_id = mj_name2id(m, mjOBJ_HFIELD, "terrain");
    bucket_soil_1_id = mj_name2id(m, mjOBJ_HFIELD, "bucket soil 1");
    bucket_soil_2_id = mj_name2id(m, mjOBJ_HFIELD, "bucket soil 2");
    soil_id = mj_name2id(m, mjOBJ_PLUGIN, "terrain");
    if ((terrain_id != -1) && (soil_id != -1)) {
      soil_plugin = true;
      int spec = mjSTATE_PLUGIN;
      int size = mj_stateSize(m, spec);
      std::vector<mjtNum> soil_state(size);
      mj_getState(m, d, soil_state.data(), spec);
      if (soil_state.size() != 1) {
        mju_warning("Too many plugin state, disabling visual update");
        soil_plugin = false;
      }
    } else {
      soil_plugin = false;
    }

    {
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      if (m) {
        if (sim.run) {
          const auto startCPU = mj::Simulate::Clock::now();
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // ===== ROS2: 写入执行器控制值 =====
          if (cmd_received) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            for (int i = 0; i < 4; ++i) {
              if (act_ids[i] >= 0) {
                d->ctrl[act_ids[i]] = cmd_buffer[i];
              }
            }
          }

          // inject noise（如果启用了噪声，会覆盖上面的值）
          if (sim.ctrl_noise_std) {
            mjtNum rate = mju_exp(
              -m->opt.timestep / mju_max(sim.ctrl_noise_rate, mjMINVAL));
            mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1-rate*rate);
            for (int i = 0; i < m->nu; i++) {
              ctrlnoise[i] = (rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr));
              d->ctrl[i] = ctrlnoise[i];
            }
          }

          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];
          bool misaligned =
              mju_abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
              syncCPU.time_since_epoch().count() == 0 || misaligned || sim.speed_changed) {
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;
            mj_step(m, d);
          } else {
            bool measured = false;
            mjtNum prevSim = d->time;
            double refreshTime = simRefreshFraction/sim.refresh_rate;
            while (
                Seconds((d->time - syncSim)*slowdown) <
                    mj::Simulate::Clock::now() - syncCPU &&
                mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }
              mj_step(m, d);
              if (d->time < prevSim) break;
            }
          }

          // ===== ROS2: 发布关节状态 =====
          publish_joint_states();

        } else {
          mj_forward(m, d);
        }
      }
    }

    if (soil_plugin) {
      int spec = mjSTATE_PLUGIN;
      int size = mj_stateSize(m, spec);
      std::vector<mjtNum> soil_state(size);
      mj_getState(m, d, soil_state.data(), spec);
      if (soil_state[0] == 1.0) {
        sim.UpdateHField(terrain_id);
        sim.UpdateHField(bucket_soil_1_id);
        sim.UpdateHField(bucket_soil_2_id);
      }
    }
  }
}
}  // namespace

//---------------------------- physics_thread ----------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  std::printf("[Debug] PhysicsThread启动\n"); fflush(stdout);

  if (filename != nullptr) {
    std::printf("[Debug] 加载模型: %s\n", filename); fflush(stdout);
    m = LoadModel(filename, *sim);
    std::printf("[Debug] LoadModel返回: %p\n", (void*)m); fflush(stdout);
    if (m) d = mj_makeData(m);
    std::printf("[Debug] mj_makeData返回: %p\n", (void*)d); fflush(stdout);
    if (d) {
      std::printf("[Debug] sim->Load...\n"); fflush(stdout);
      sim->Load(m, d, filename);
      std::printf("[Debug] mj_forward...\n"); fflush(stdout);
      mj_forward(m, d);

      free(ctrlnoise);
      ctrlnoise = static_cast<mjtNum*>(malloc(sizeof(mjtNum)*m->nu));
      mju_zero(ctrlnoise, m->nu);

      // 初始化ROS2关节映射和执行器ID
      init_joint_mapping();
    }
  }

  PhysicsLoop(*sim);

  free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//-------------------------------- main ----------------------------------------

int main(int argc, const char** argv) {
  // 在ROS2初始化之前保存模型路径（rclcpp::init会修改argc/argv）
  const char* filename = nullptr;
  if (argc > 1) filename = argv[1];

  // 初始化ROS2
  rclcpp::init(argc, const_cast<char**>(argv));
  ros_node = std::make_shared<rclcpp::Node>("excavator_simulate");

  // 订阅velocity命令
  auto sub = ros_node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/commands", 10, velocity_command_callback);

  std::printf("[ROS2] 节点已初始化，订阅 /velocity_controller/commands\n");

  // 启动ROS2 spinner线程
  std::thread ros_thread([]() {
    rclcpp::spin(ros_node);
  });

  // 打印版本
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // 加载插件
  scanPluginLibraries();

  // 初始化MuJoCo可视化
  mjvScene scn;
  mjv_defaultScene(&scn);
  mjvCamera cam;
  mjv_defaultCamera(&cam);
  mjvOption opt;
  mjv_defaultOption(&opt);
  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &scn, &cam, &opt, &pert, /* fully_managed = */ true);

  std::printf("[ROS2] 模型路径: %s\n", filename ? filename : "(null)"); fflush(stdout);

  // 启动物理线程
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  std::printf("[Debug] 启动渲染循环...\n"); fflush(stdout);

  // 主线程运行渲染循环（阻塞）
  sim->RenderLoop();

  // 清理
  physicsthreadhandle.join();
  rclcpp::shutdown();
  ros_thread.join();

  return 0;
}
