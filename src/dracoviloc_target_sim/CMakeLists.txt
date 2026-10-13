cmake_minimum_required(VERSION 3.8)
project(dracoviloc_target_sim)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(target_sim_node src/target_sim_node.cpp)
ament_target_dependencies(target_sim_node geometry_msgs rclcpp visualization_msgs)

install(TARGETS target_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
