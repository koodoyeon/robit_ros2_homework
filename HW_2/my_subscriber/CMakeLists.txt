cmake_minimum_required(VERSION 3.5)
project(my_subscriber)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_interfaces REQUIRED)

add_executable(subscriber_node src/subscriber_node.cpp)
ament_target_dependencies(subscriber_node rclcpp my_interfaces)

install(TARGETS
  subscriber_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

