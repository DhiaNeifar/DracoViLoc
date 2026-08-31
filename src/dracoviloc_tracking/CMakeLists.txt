cmake_minimum_required(VERSION 3.8)
project(dracoviloc_tracking)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(odas_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(arm_drone_tracker src/arm_drone_tracker.cpp)
ament_target_dependencies(arm_drone_tracker
  geometry_msgs moveit_ros_planning_interface rclcpp sensor_msgs tf2 tf2_eigen tf2_ros
  visualization_msgs)
target_link_libraries(arm_drone_tracker Eigen3::Eigen)

add_executable(arm_audio_tracker src/arm_audio_tracker.cpp)
ament_target_dependencies(arm_audio_tracker
  geometry_msgs odas_ros_msgs rclcpp sensor_msgs std_msgs tf2_ros trajectory_msgs)
target_link_libraries(arm_audio_tracker Eigen3::Eigen)

install(TARGETS arm_drone_tracker arm_audio_tracker DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
