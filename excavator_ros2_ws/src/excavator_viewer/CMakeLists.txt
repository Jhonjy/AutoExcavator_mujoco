cmake_minimum_required(VERSION 3.8)
project(excavator_viewer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== ROS2 依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ==================== MuJoCo 路径 ====================
set(EXCAVATOR_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(MUJOCO_PROJECT_DIR "${EXCAVATOR_WS_ROOT}/excavator_simulator_mujoco")
set(MUJOCO_BUILD_DIR "${MUJOCO_PROJECT_DIR}/build")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_BUILD_DIR}/_deps/mujoco-src/include")
set(MUJOCO_LIB_DIR "${MUJOCO_BUILD_DIR}/lib")
set(GLFW_INCLUDE_DIR "${MUJOCO_BUILD_DIR}/_deps/glfw3-src/include")

# 查找OpenGL
find_package(OpenGL REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)

# 查找X11 (GLFW需要)
find_package(X11 REQUIRED)

# ==================== 查看器可执行文件 ====================
add_executable(mujoco_viewer src/mujoco_viewer.cpp)

target_include_directories(mujoco_viewer PRIVATE
  ${MUJOCO_INCLUDE_DIR}
  ${GLFW_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
)

target_link_directories(mujoco_viewer PRIVATE ${MUJOCO_LIB_DIR})

target_link_libraries(mujoco_viewer
  mujoco
  glfw3
  OpenGL::GL
  ${X11_LIBRARIES}
  Xcursor
  Xrandr
  Xinerama
  Xi
  pthread
  dl
)

set_target_properties(mujoco_viewer PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
)

target_compile_definitions(mujoco_viewer PRIVATE
  EXCAVATOR_WS_ROOT="${EXCAVATOR_WS_ROOT}"
)
ament_target_dependencies(mujoco_viewer rclcpp std_msgs)

# ==================== 安装 ====================
install(TARGETS mujoco_viewer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
