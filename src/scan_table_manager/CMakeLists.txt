cmake_minimum_required(VERSION 3.5)
project(scan_table_manager)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(scan_table_manager src/scan_table_manager.cpp)
ament_target_dependencies(scan_table_manager rclcpp std_msgs)

install(TARGETS
  scan_table_manager
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()
