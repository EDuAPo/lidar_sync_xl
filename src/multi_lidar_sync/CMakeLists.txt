cmake_minimum_required(VERSION 3.8)
project(multi_lidar_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies (minimal - no PCL required)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(multi_lidar_sync_node
  src/lidar_buffer_manager.cpp
  src/lidar_processor.cpp
  src/multi_lidar_processor.cpp
  src/multi_lidar_sync_node.cpp
)

# Link dependencies
ament_target_dependencies(multi_lidar_sync_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install targets
install(TARGETS
  multi_lidar_sync_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
