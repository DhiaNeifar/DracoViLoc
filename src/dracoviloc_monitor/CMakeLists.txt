cmake_minimum_required(VERSION 3.8)
project(dracoviloc_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dracoviloc_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(odas_ros_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

add_executable(preview_node src/preview_node.cpp)
target_include_directories(preview_node PRIVATE include)
ament_target_dependencies(preview_node rclcpp sensor_msgs vision_msgs)
target_link_libraries(preview_node ${OpenCV_LIBS})

add_executable(telemetry_node src/telemetry_node.cpp)
target_include_directories(telemetry_node PRIVATE include)
ament_target_dependencies(telemetry_node
  dracoviloc_msgs geometry_msgs odas_ros_msgs rclcpp sensor_msgs std_msgs vision_msgs)

install(TARGETS preview_node telemetry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_include_directories(test_kinematics PRIVATE include)
  ament_add_gtest(test_detection_geometry test/test_detection_geometry.cpp)
  target_include_directories(test_detection_geometry PRIVATE include)
endif()

ament_package()
