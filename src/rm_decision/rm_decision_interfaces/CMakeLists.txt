cmake_minimum_required(VERSION 3.8)
project(rm_decision_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AllRobotHP.msg"
  "msg/FriendLocation.msg"
  "msg/GameStatus.msg"
  "msg/RobotStatus.msg"
  "msg/RobotEconomy.msg"
  "msg/DecisionNum.msg"
  "msg/RobotControl.msg"
  "msg/RFID.msg"
  "msg/Controller.msg"
  "msg/RobotLocation.msg"

  "msg/SentryPostureCmd.msg"
  "msg/SentryPostureStatus.msg"
  "msg/Target.msg"
  "msg/ObservationPoint.msg"
  "msg/ObservationPoints.msg"
  "msg/GoalStatus.msg"
  "srv/SetSentryPosture.srv"
  "msg/RFIDParse.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
    builtin_interfaces
)



ament_package()
