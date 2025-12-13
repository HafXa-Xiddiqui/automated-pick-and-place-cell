cmake_minimum_required(VERSION 3.5)
project(scan_table_manager)

# Require C++17 for std::chrono_literals
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(scan_table_manager src/scan_table_manager.cpp)

# Link dependencies
ament_target_dependencies(scan_table_manager
  rclcpp
  std_msgs
)

# Install the executable
install(TARGETS
  scan_table_manager
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
