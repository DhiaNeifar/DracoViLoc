cmake_minimum_required(VERSION 3.8)
project(dracoviloc_bringup)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(arm_keyboard_teleop src/arm_keyboard_teleop.cpp)
ament_target_dependencies(arm_keyboard_teleop
  rclcpp sensor_msgs trajectory_msgs)

install(DIRECTORY launch config worlds DESTINATION share/${PROJECT_NAME})
install(TARGETS arm_keyboard_teleop DESTINATION lib/${PROJECT_NAME})
ament_package()
