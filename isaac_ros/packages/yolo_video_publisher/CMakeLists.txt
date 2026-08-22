cmake_minimum_required(VERSION 3.8)
project(yolo_video_publisher)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc videoio)

add_executable(video_publisher_node src/video_publisher_node.cpp)
target_compile_features(video_publisher_node PUBLIC cxx_std_17)
ament_target_dependencies(video_publisher_node rclcpp sensor_msgs)
target_include_directories(video_publisher_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(video_publisher_node ${OpenCV_LIBRARIES})

install(TARGETS video_publisher_node DESTINATION lib/${PROJECT_NAME})

ament_package()
