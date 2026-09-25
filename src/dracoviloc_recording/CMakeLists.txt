cmake_minimum_required(VERSION 3.8)
project(dracoviloc_recording)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(audio_utils_msgs REQUIRED)
find_package(odas_ros_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(multimodal_recorder src/multimodal_recorder.cpp)
ament_target_dependencies(multimodal_recorder
  audio_utils_msgs
  odas_ros_msgs
  rclcpp
  sensor_msgs)

add_executable(sss_channel_recorder src/sss_channel_recorder.cpp)
ament_target_dependencies(sss_channel_recorder
  audio_utils_msgs
  rclcpp)

install(TARGETS multimodal_recorder sss_channel_recorder DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
