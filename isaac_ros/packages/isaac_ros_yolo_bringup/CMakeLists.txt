cmake_minimum_required(VERSION 3.8)
project(isaac_ros_yolo_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/yolo_visualizer.py DESTINATION lib/${PROJECT_NAME})

ament_package()
