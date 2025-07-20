cmake_minimum_required(VERSION 3.22)
project(force_emergency_stop)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)


add_executable(force_emergency_stop src/force_emergency_stop.cpp)

ament_target_dependencies(force_emergency_stop
  rclcpp
  rclcpp_action
  std_srvs
  moveit_msgs
  control_msgs  
  moveit_ros_planning_interface
  geometry_msgs
)

install(TARGETS
  force_emergency_stop
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
