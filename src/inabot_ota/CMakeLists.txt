cmake_minimum_required(VERSION 3.5)
project(inabot_ota)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  include/ota_node      
)

include_directories(
  include
  include/cognex_is8905      
)

add_executable(ota_node
  src/main.cpp
  src/ota_node.cpp
)

ament_target_dependencies(ota_node
  rclcpp
  std_msgs
)

install(TARGETS ota_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 스크립트 설치
install(
  PROGRAMS
    script/exit_all.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

