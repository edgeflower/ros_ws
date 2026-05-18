cmake_minimum_required(VERSION 3.8)
project(rm_set_posture)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rm_decision_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 共享库（组件模式）
add_library(${PROJECT_NAME}_component SHARED src/set_posture_node.cpp)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_definitions(${PROJECT_NAME}_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  rm_decision_interfaces
  std_msgs
  geometry_msgs
)
rclcpp_components_register_nodes(${PROJECT_NAME}_component "SetPostureNode")

# 独立可执行文件（链接组件共享库）
add_executable(set_posture_node src/standalone.cpp)
target_include_directories(set_posture_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(set_posture_node ${PROJECT_NAME}_component)
ament_target_dependencies(set_posture_node
  rclcpp
  rclcpp_components
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_component
  DESTINATION lib
)

install(TARGETS set_posture_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
