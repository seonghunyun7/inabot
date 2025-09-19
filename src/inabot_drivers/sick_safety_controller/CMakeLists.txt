cmake_minimum_required(VERSION 3.5)
project(sick_safety_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(inabot_msgs REQUIRED)

# ------------------------------
# libmodbus 찾기
# ------------------------------
pkg_check_modules(MODBUS REQUIRED libmodbus)
include_directories(${MODBUS_INCLUDE_DIRS})

include_directories(include)

add_executable(safety_controller_node
  src/main.cpp
  src/safety_controller.cpp
)

ament_target_dependencies(safety_controller_node
  rclcpp
  std_msgs
  inabot_msgs
)

# ------------------------------
# libmodbus 라이브러리 링크
# ------------------------------
target_link_libraries(safety_controller_node ${MODBUS_LIBRARIES})

install(TARGETS
  safety_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
