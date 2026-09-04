cmake_minimum_required(VERSION 3.8)
project(isaac_ros_yolo_bringup)

find_package(ament_cmake REQUIRED)
find_package(ALSA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(multimodal_recorder src/multimodal_recorder.cpp)
ament_target_dependencies(multimodal_recorder rclcpp sensor_msgs)
target_include_directories(multimodal_recorder PRIVATE ${ALSA_INCLUDE_DIRS})
target_link_libraries(multimodal_recorder ${OpenCV_LIBRARIES} ${ALSA_LIBRARIES})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/yolo_visualizer.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS multimodal_recorder DESTINATION lib/${PROJECT_NAME})

ament_package()
