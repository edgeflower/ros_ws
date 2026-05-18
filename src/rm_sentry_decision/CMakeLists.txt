cmake_minimum_required(VERSION 3.8)
project(rm_sentry_decision)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 强制使用 C++17 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rm_decision_interfaces REQUIRED)
find_package(armor_interfaces REQUIRED)

# 编译 utility_ai_node 组件库，并生成同名可执行文件
add_library(utility_ai_node_component SHARED
  src/utility_ai_node.cpp
)

# 设置头文件包含路径
target_include_directories(utility_ai_node_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# 链接依赖
ament_target_dependencies(utility_ai_node_component
  rclcpp
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rm_decision_interfaces
  armor_interfaces
)

rclcpp_components_register_node(utility_ai_node_component
  PLUGIN "UtilityAiNode"
  EXECUTABLE utility_ai_node
)

# 安装可执行文件和组件库
install(TARGETS utility_ai_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS utility_ai_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装配置文件
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 安装 launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
