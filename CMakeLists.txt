cmake_minimum_required(VERSION 3.22)
project(force_emergency_stop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(force_emergency_stop src/force_emergency_stop.cpp)
add_executable(servo_stop src/servo_stop.cpp)
add_executable(key_action_cancel src/key_action_cancel.cpp)

ament_target_dependencies(
  force_emergency_stop
  rclcpp
  rclcpp_action
  std_srvs
  moveit_msgs
  control_msgs  
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(
  servo_stop
  rclcpp
  geometry_msgs
)

ament_target_dependencies(key_action_cancel
  rclcpp rclcpp_action control_msgs)

install(TARGETS key_action_cancel
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  force_emergency_stop
  servo_stop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
