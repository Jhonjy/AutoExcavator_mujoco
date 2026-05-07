cmake_minimum_required(VERSION 3.16)
project(excavator_simulate_ros2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 原始项目路径
set(MUJOCO_BUILD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../excavator_simulator_mujoco/build")
set(MUJOCO_SRC_DIR "${MUJOCO_BUILD_DIR}/_deps/mujoco-src")

# 所有库都在同一个目录下
set(LIB_DIR "${MUJOCO_BUILD_DIR}/lib")

add_executable(simulate_ros2 src/main.cc)

target_include_directories(simulate_ros2 PRIVATE
  "${MUJOCO_SRC_DIR}/include"
  "${MUJOCO_SRC_DIR}/simulate"
  "${MUJOCO_SRC_DIR}/sample"
  "${MUJOCO_BUILD_DIR}/_deps/lodepng-src"
)

target_link_libraries(simulate_ros2
  "${LIB_DIR}/libsimulate.a"
  "${LIB_DIR}/libmujoco.so"
  "${LIB_DIR}/libglfw3.a"
  "${LIB_DIR}/liblodepng.a"
  Threads::Threads
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
)

# OpenGL
find_package(OpenGL REQUIRED)
target_link_libraries(simulate_ros2 OpenGL::GL)

# X11 (Linux)
find_package(X11 REQUIRED)
target_link_libraries(simulate_ros2 ${X11_LIBRARIES})

# 运行时库路径
target_link_directories(simulate_ros2 PRIVATE "${LIB_DIR}")

# 设置rpath让运行时能找到libmujoco.so
set_target_properties(simulate_ros2 PROPERTIES
  INSTALL_RPATH "${LIB_DIR}"
  BUILD_RPATH "${LIB_DIR}"
)

install(TARGETS simulate_ros2 DESTINATION lib/${PROJECT_NAME})

ament_package()
