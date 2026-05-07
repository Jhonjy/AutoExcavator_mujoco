cmake_minimum_required(VERSION 3.8)
project(excavator_ros2_bridge)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== ROS2 依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==================== MuJoCo 路径配置 ====================
# 引用原始excavator_simulator_mujoco项目的构建产物，不重新编译MuJoCo
# CMAKE_CURRENT_SOURCE_DIR = .../excavator_ros2_ws/src/excavator_ros2_bridge
# 需要往上4级到mujoco_develop
set(EXCAVATOR_WS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../..")
set(MUJOCO_PROJECT_DIR "${EXCAVATOR_WS_ROOT}/excavator_simulator_mujoco")
set(MUJOCO_BUILD_DIR "${MUJOCO_PROJECT_DIR}/build")
set(MUJOCO_INCLUDE_DIR "${MUJOCO_BUILD_DIR}/_deps/mujoco-src/include")
set(MUJOCO_LIB_DIR "${MUJOCO_BUILD_DIR}/lib")

# 验证MuJoCo路径存在
if(NOT EXISTS "${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
  message(FATAL_ERROR "MuJoCo头文件未找到: ${MUJOCO_INCLUDE_DIR}/mujoco/mujoco.h")
endif()
if(NOT EXISTS "${MUJOCO_LIB_DIR}/libmujoco.so")
  message(FATAL_ERROR "MuJoCo库文件未找到: ${MUJOCO_LIB_DIR}/libmujoco.so")
endif()

# ==================== 硬件接口库 (pluginlib插件) ====================
add_library(excavator_hardware SHARED
  src/mujoco_wrapper.cpp
  src/excavator_hardware_interface.cpp
)
target_include_directories(excavator_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MUJOCO_INCLUDE_DIR}
)
target_link_directories(excavator_hardware PUBLIC ${MUJOCO_LIB_DIR})
target_link_libraries(excavator_hardware mujoco)
# 确保运行时能找到libmujoco.so
set_target_properties(excavator_hardware PROPERTIES
  INSTALL_RPATH "${MUJOCO_LIB_DIR}"
  BUILD_RPATH "${MUJOCO_LIB_DIR}"
)

target_compile_definitions(excavator_hardware PRIVATE
  EXCAVATOR_WS_ROOT="${EXCAVATOR_WS_ROOT}"
)
ament_target_dependencies(excavator_hardware
  hardware_interface
  pluginlib
  rclcpp
)

# ==================== 主节点可执行文件 ====================
add_executable(bridge_node src/bridge_node_main.cpp)
target_include_directories(bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(bridge_node excavator_hardware)
ament_target_dependencies(bridge_node
  rclcpp
  controller_manager
  hardware_interface
)

# ==================== 测试可执行文件 ====================
set(MUJOCO_PLUGIN_DIR "${MUJOCO_PROJECT_DIR}/build/bin/mujoco_plugin")

add_executable(test_mujoco_model test_mujoco_model.cpp)
target_include_directories(test_mujoco_model PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_directories(test_mujoco_model PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(test_mujoco_model mujoco)
target_compile_definitions(test_mujoco_model PRIVATE
  EXCAVATOR_WS_ROOT="${EXCAVATOR_WS_ROOT}"
)

add_executable(test_mujoco_ros2 test_mujoco_ros2.cpp)
target_include_directories(test_mujoco_ros2 PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_directories(test_mujoco_ros2 PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(test_mujoco_ros2 mujoco)
target_compile_definitions(test_mujoco_ros2 PRIVATE
  EXCAVATOR_WS_ROOT="${EXCAVATOR_WS_ROOT}"
)

add_executable(test_viewer_logic test_viewer_logic.cpp)
target_include_directories(test_viewer_logic PRIVATE ${MUJOCO_INCLUDE_DIR})
target_link_directories(test_viewer_logic PRIVATE ${MUJOCO_LIB_DIR})
target_link_libraries(test_viewer_logic mujoco)
target_compile_definitions(test_viewer_logic PRIVATE
  EXCAVATOR_WS_ROOT="${EXCAVATOR_WS_ROOT}"
)

# ==================== 安装 ====================
install(TARGETS excavator_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config description
  DESTINATION share/${PROJECT_NAME}
)

# pluginlib描述文件
pluginlib_export_plugin_description_file(hardware_interface excavator_hardware.xml)

ament_package()
