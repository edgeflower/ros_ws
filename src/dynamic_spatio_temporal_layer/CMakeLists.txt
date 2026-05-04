cmake_minimum_required(VERSION 3.8)
project(dynamic_spatio_temporal_layer)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(openvdb_vendor REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(message_filters REQUIRED)

set(STVL_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../spatio_temporal_voxel_layer/spatio_temporal_voxel_layer")

add_library(${PROJECT_NAME} SHARED
  src/dynamic_spatio_temporal_layer.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${STVL_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenVDB_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${laser_geometry_INCLUDE_DIRS}
  /opt/ros/humble/include/pcl_conversions
  /opt/ros/humble/include/pcl_msgs
  /opt/ros/humble/include/spatio_temporal_voxel_layer
  /opt/ros/humble/include/sensor_msgs
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${OpenVDB_LIBRARIES}
  /opt/ros/humble/lib/libspatio_temporal_voxel_layer_core.so
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp nav2_costmap_2d nav2_util sensor_msgs nav_msgs
  tf2 tf2_ros tf2_eigen pluginlib message_filters laser_geometry
)

target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wpedantic
  $<$<CXX_COMPILER_ID:GNU>:-Wno-deprecated-copy>
)

pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES costmap_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY cfg/ DESTINATION share/${PROJECT_NAME}/cfg)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp nav2_costmap_2d sensor_msgs nav_msgs tf2 tf2_ros tf2_eigen pluginlib
)

ament_package()
