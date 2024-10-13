cmake_minimum_required(VERSION 3.5)
project(my_publisher)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)

add_executable(publisher_node src/publisher_node.cpp)
ament_target_dependencies(publisher_node rclcpp my_interfaces)

install(TARGETS
  publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

