cmake_minimum_required(VERSION 3.8)
project(cognex_is8905)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  include/cognex_is8905      
)

add_executable(cognex_is8905_node
  src/main.cpp
  src/cognex_is8905.cpp
)

ament_target_dependencies(cognex_is8905_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  cognex_is8905_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()