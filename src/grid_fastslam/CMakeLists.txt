cmake_minimum_required(VERSION 3.8)
project(grid_fastslam)

# Standard C++17 for ROS 2
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(custom_msgs REQUIRED)

# 2. Include Header Directories
# Tells CMake where to look for grid_fast_slam.hpp
include_directories(include)

# 3. Add Executable
# This combines your logic (.cpp) and the entry point (main.cpp) into a binary
add_executable(grid_fastslam_node
  src/grid_fastslam.cpp
  src/main.cpp
)

# 4. Link Dependencies
# Links the libraries found in step 1 to your executable
ament_target_dependencies(grid_fastslam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  custom_msgs
)

# 5. Install Targets
# Places the executable in the install folder so 'ros2 run' can find it
install(TARGETS grid_fastslam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Installs headers (good practice)
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()