cmake_minimum_required(VERSION 3.8)
project(dracoviloc_tracking)

find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(odas_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(arm_tracker src/arm_tracker.cpp)
ament_target_dependencies(arm_tracker
  controller_manager_msgs geometry_msgs odas_ros_msgs rclcpp sensor_msgs std_msgs
  std_srvs tf2_ros trajectory_msgs)
target_link_libraries(arm_tracker Eigen3::Eigen ruckig::ruckig)

install(TARGETS arm_tracker DESTINATION lib/${PROJECT_NAME})
ament_package()
