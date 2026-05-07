// MuJoCo挖掘机可视化查看器
// 独立运行MuJoCo物理仿真，订阅velocity命令控制挖掘机
// 与bridge的MuJoCo实例独立，接收相同的velocity命令
//
// 用法：
//   ros2 run excavator_viewer mujoco_viewer [model.xml]

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// ==================== 全局变量 ====================

mjModel* m = nullptr;
mjData* d = nullptr;

mjvCamera cam;
mjvOption opt;
mjvPerturb pert;
mjvScene scn;
mjrContext con;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0.0;
double lasty = 0.0;

std::mutex cmd_mutex;
double cmd_buffer[4] = {0.0, 0.0, 0.0, 0.0};
bool cmd_received = false;

int mj_act_ids[4] = {-1, -1, -1, -1};

// ==================== GLFW回调 ====================

void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  (void)scancode;
  (void)mods;
  if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, GLFW_TRUE);
  }
}

void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  (void)mods;
  button_left =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right =
      (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  if (!button_left && !button_middle && !button_right) return;

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift =
      (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) ||
      (glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  (void)window;
  (void)xoffset;
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// ==================== ROS2回调 ====================

void velocity_command_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
  if (msg->data.size() >= 4) {
    std::lock_guard<std::mutex> lock(cmd_mutex);
    for (int i = 0; i < 4; ++i) {
      cmd_buffer[i] = std::clamp(msg->data[i], -0.5, 0.5);
    }
    cmd_received = true;
  }
}

// ==================== 插件加载 ====================

void load_plugins() {
  std::string plugin_dir =
      "/home/ubuntu2204/mujoco_develop/"
      "excavator_simulator_mujoco/build/bin/mujoco_plugin";

  std::cout << "[Viewer] 加载插件目录: " << plugin_dir << std::endl;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(),
      +[](const char* filename, int first, int count) {
        std::cout << "[Viewer] 插件库 '" << filename << "' 注册了 " << count
                  << " 个插件" << std::endl;
        for (int i = first; i < first + count; ++i) {
          std::cout << "  - " << mjp_getPluginAtSlot(i)->name << std::endl;
        }
      });
}

// ==================== 模型加载 ====================

bool load_model(const std::string& model_path) {
  char error[1024] = "";
  m = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!m) {
    std::cerr << "[Viewer] 模型加载失败: " << error << std::endl;
    return false;
  }

  d = mj_makeData(m);
  if (!d) {
    std::cerr << "[Viewer] 仿真数据创建失败" << std::endl;
    mj_deleteModel(m);
    m = nullptr;
    return false;
  }

  // 查找执行器ID
  static const std::vector<std::string> act_names = {
      "Rotation", "Boom", "Arm", "Bucket"};
  for (size_t i = 0; i < act_names.size(); ++i) {
    mj_act_ids[i] = mj_name2id(m, mjOBJ_ACTUATOR, act_names[i].c_str());
    if (mj_act_ids[i] < 0) {
      std::cerr << "[Viewer] 警告: 执行器 '" << act_names[i] << "' 未找到"
                << std::endl;
    } else {
      std::cout << "[Viewer] 执行器: " << act_names[i] << " -> ID "
                << mj_act_ids[i] << std::endl;
    }
  }

  // 初始化物理（qpos0已在mj_makeData中设置）
  mj_forward(m, d);

  std::cout << "[Viewer] 模型加载成功: " << model_path << std::endl;
  std::cout << "  关节数: " << m->njnt << std::endl;
  std::cout << "  执行器数: " << m->nu << std::endl;

  return true;
}

// ==================== 物理步进 ====================

void physics_step() {
  std::lock_guard<std::mutex> lock(cmd_mutex);

  // 设置执行器控制值（与bridge接收相同的velocity命令）
  for (int i = 0; i < 4; ++i) {
    if (mj_act_ids[i] >= 0) {
      d->ctrl[mj_act_ids[i]] = cmd_buffer[i];
    }
  }

  // 物理步进（包含约束求解、碰撞检测、土壤变形）
  mj_step(m, d);

  // 检查土壤插件，更新hfield渲染
  int soil_plugin_id = mj_name2id(m, mjOBJ_PLUGIN, "terrain");
  if (soil_plugin_id >= 0) {
    int spec = mjSTATE_PLUGIN;
    int size = mj_stateSize(m, spec);
    std::vector<mjtNum> soil_state(size);
    mj_getState(m, d, soil_state.data(), spec);
    if (soil_state[0] == 1.0) {
      int terrain_hf = mj_name2id(m, mjOBJ_HFIELD, "terrain");
      int soil1_hf = mj_name2id(m, mjOBJ_HFIELD, "bucket soil 1");
      int soil2_hf = mj_name2id(m, mjOBJ_HFIELD, "bucket soil 2");
      if (terrain_hf >= 0) mjr_uploadHField(m, &con, terrain_hf);
      if (soil1_hf >= 0) mjr_uploadHField(m, &con, soil1_hf);
      if (soil2_hf >= 0) mjr_uploadHField(m, &con, soil2_hf);
      soil_state[0] = 0.0;
      mj_setState(m, d, soil_state.data(), spec);
    }
  }
}

// ==================== 主函数 ====================

int main(int argc, char** argv) {
  const char* filename = nullptr;
  if (argc > 1) filename = argv[1];

  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("mujoco_viewer");

  // 订阅velocity命令（与bridge相同的topic）
  auto sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/velocity_controller/commands", 10, velocity_command_callback);

  std::cout << "[Viewer] 订阅 /velocity_controller/commands" << std::endl;

  load_plugins();

  std::string model_path =
      "/home/ubuntu2204/mujoco_develop/"
      "excavator_ros2_ws/src/excavator_ros2_bridge/config/excavator_control.xml";
  if (filename) model_path = filename;

  if (!load_model(model_path)) return 1;

  if (!glfwInit()) {
    std::cerr << "[Viewer] GLFW初始化失败" << std::endl;
    return 1;
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  GLFWwindow* window =
      glfwCreateWindow(1200, 900, "MuJoCo 挖掘机仿真", nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    std::cerr << "[Viewer] GLFW窗口创建失败" << std::endl;
    return 1;
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultPerturb(&pert);
  mjr_defaultContext(&con);

  cam.distance = 8.0;
  cam.azimuth = 135.0;
  cam.elevation = -20.0;
  cam.lookat[0] = 0.0;
  cam.lookat[1] = 0.0;
  cam.lookat[2] = 1.0;

  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);
  scn.flags[mjRND_WIREFRAME] = 1;

  std::cout << "\n[Viewer] ==============================" << std::endl;
  std::cout << "[Viewer] MuJoCo 挖掘机可视化查看器" << std::endl;
  std::cout << "[Viewer] ==============================" << std::endl;
  std::cout << "[Viewer] 操作说明:" << std::endl;
  std::cout << "[Viewer]   鼠标左键拖动: 旋转视角" << std::endl;
  std::cout << "[Viewer]   鼠标右键拖动: 平移视角" << std::endl;
  std::cout << "[Viewer]   滚轮: 缩放" << std::endl;
  std::cout << "[Viewer]   ESC: 退出" << std::endl;
  std::cout << "[Viewer] ==============================" << std::endl;

  // 主循环
  while (!glfwWindowShouldClose(window)) {
    rclcpp::spin_some(node);

    // 物理步进（与bridge相同的逻辑）
    physics_step();

    // 渲染
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    mjv_updateScene(m, d, &opt, &pert, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // 状态显示
    {
      char status[256];
      if (cmd_received) {
        snprintf(status, sizeof(status),
                 "ctrl: [%.3f, %.3f, %.3f, %.3f]",
                 cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3]);
      } else {
        snprintf(status, sizeof(status), "等待 /velocity_controller/commands ...");
      }
      mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport, status, nullptr,
                  &con);
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);
  glfwTerminate();
  rclcpp::shutdown();

  return 0;
}
