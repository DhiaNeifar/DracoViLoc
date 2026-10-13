cmake_minimum_required(VERSION 3.8)
project(dracoviloc_diagnostics)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(event_logger_node src/event_logger_node.cpp)
ament_target_dependencies(event_logger_node
  geometry_msgs rclcpp sensor_msgs std_msgs tf2_ros)

install(TARGETS event_logger_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/live_plot.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
