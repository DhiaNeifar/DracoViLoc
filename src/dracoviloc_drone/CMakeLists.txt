cmake_minimum_required(VERSION 3.8)
project(dracoviloc_drone)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(drone_pose_controller src/drone_pose_controller.cpp)
ament_target_dependencies(drone_pose_controller
  geometry_msgs rclcpp tf2 tf2_ros)
target_link_libraries(drone_pose_controller
  ignition-msgs8::ignition-msgs8
  ignition-transport11::ignition-transport11)

add_executable(drone_keyboard_teleop src/drone_keyboard_teleop.cpp)
ament_target_dependencies(drone_keyboard_teleop geometry_msgs rclcpp)

install(TARGETS drone_pose_controller drone_keyboard_teleop
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY urdf launch config DESTINATION share/${PROJECT_NAME})
ament_package()
