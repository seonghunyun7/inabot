cmake_minimum_required(VERSION 3.5)
project(inabot_acs)

## Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# fmt 라이브러리
set(fmt_LIBRARIES fmt::fmt)
set(CMAKE_LIBRARY_PATH "/usr/local/lib" ${CMAKE_LIBRARY_PATH})

# Paho MQTT C++/C 라이브러리
find_library(PahoMqttC_LIBRARY NAMES libpaho-mqtt3as.so REQUIRED)
find_library(PahoMqttCpp_LIBRARY NAMES libpaho-mqttpp3.so REQUIRED)

## Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)  # JSON 라이브러리
find_package(Boost REQUIRED COMPONENTS
  log
  log_setup
  thread
  filesystem
  date_time
  system
  stacktrace_backtrace
  regex
)

## Include directories
include_directories(
  include
  include/inabot_acs
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/json/single_include/  # nlohmann/json
  ${fmt_INCLUDE_DIRS}
)

## Main executable
add_executable(robot_acs_node
  src/main.cpp
  src/acs_node.cpp
  src/heartbeat_manager.cpp
  src/mqtt_client.cpp
  src/message_handler.cpp
  src/mission_manager.cpp
)

target_link_libraries(robot_acs_node
  ${PahoMqttC_LIBRARY}
  ${PahoMqttCpp_LIBRARY}
  ${fmt_LIBRARIES}
  Boost::log
  Boost::log_setup
  Boost::thread
  Boost::system
  Boost::filesystem
  Boost::date_time
  Boost::stacktrace_backtrace
)

## Add dependencies
ament_target_dependencies(robot_acs_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_geometry_msgs
  nlohmann_json
)

## Install targets
install(TARGETS
  robot_acs_node
  DESTINATION lib/${PROJECT_NAME}
)

## Install headers
install(DIRECTORY include/
  DESTINATION include/
)

## Install launch files (if any)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
