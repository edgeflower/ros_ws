cmake_minimum_required(VERSION 3.8)
project(cloud_rgb_to_intensity)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(cloud_rgb_to_intensity_node
  src/cloud_rgb_to_intensity.cpp
)

ament_target_dependencies(cloud_rgb_to_intensity_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(cloud_rgb_to_intensity_node
  ${PCL_LIBRARIES}
)

install(TARGETS cloud_rgb_to_intensity_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
