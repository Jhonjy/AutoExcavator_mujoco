cmake_minimum_required(VERSION 3.8)
project(excavator_auto_dig)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== ROS2 依赖 ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ==================== 自动挖掘节点 ====================
add_executable(auto_dig_node src/auto_dig_node.cpp)
target_include_directories(auto_dig_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(auto_dig_node
  rclcpp
  std_srvs
  sensor_msgs
  std_msgs
)

# ==================== 安装 ====================
install(TARGETS auto_dig_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
