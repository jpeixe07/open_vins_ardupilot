cmake_minimum_required(VERSION 3.5)
project(ov_repub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories for Eigen
include_directories(${EIGEN3_INCLUDE_DIR})

# Create executable
add_executable(odom_republisher src/republisher.cpp)

# Link dependencies
ament_target_dependencies(odom_republisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# Install target
install(TARGETS
  odom_republisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
