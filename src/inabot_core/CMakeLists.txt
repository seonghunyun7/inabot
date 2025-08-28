cmake_minimum_required(VERSION 3.5)
project(inabot_core)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 옵션을 설정 (ON/OFF 또는 값 지정)
option(USE_PURE_PURSUIT "Use Pure Pursuit planner" OFF)

if(USE_PURE_PURSUIT)
  add_compile_definitions(USE_PURE_PURSUIT)
endif()

# Required ROS 2 and system packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(rcl_interfaces REQUIRED)
find_package(rcutils REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(angles REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(fmt REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)
find_package(message_filters REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(inabot_msgs REQUIRED)
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

# serial library
include_directories(/usr/local/include)
find_library(SERIAL_LIBRARY
  NAMES serial
  PATHS /usr/local/lib
  REQUIRED
)

# CPPAD & IPOPT (for MPC)
set(CPPAD_INCLUDE_DIR "/usr/local/include")
set(CPPAD_LIB_DIR "/usr/local/lib")
include_directories(${CPPAD_INCLUDE_DIR})
link_directories(${CPPAD_LIB_DIR})

# Add include directories (including all submodules)
include_directories(
  include
  include/imu
  include/kinematics
  include/monitor
  include/motion
  include/motor
  include/motor/common
  include/motor/can_driver
  include/motor/controller
  include/motor/kinematics
  include/motor/utils
  include/odom
  include/utils
  include/wheel
  include/planner/common
  include/planner/global
  include/planner/local/mpc_motion_controller
  include/planner/local/pure_pursuit
  include/sensor/lidar/scan_merger
  include/sensor/lidar/lidar_obstacle
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Automatically collect all source files
file(GLOB SOURCES
  "src/*.cpp"
  "src/imu/*.cpp"
  "src/kinematics/*.cpp"
  "src/monitor/*.cpp"
  "src/motion/*.cpp"
  "src/motor/*.cpp"
  "src/motor/can_driver/*.cpp"
  "src/motor/controller/*.cpp"
  "src/motor/kinematics/*.cpp"
  "src/motor/utils/*.cpp"
  "src/odom/*.cpp"
  "src/utils/*.cpp"
  "src/wheel/*.cpp"
  "src/planner/common/*.cpp"
  "src/planner/global/*.cpp"
  "src/planner/local/mpc_motion_controller/*.cpp"
  "src/planner/local/pure_pursuit/*.cpp"
  "src/sensor/lidar/scan_merger/*.cpp"   # <- 추가
  "src/sensor/lidar/lidar_obstacle/*.cpp"
)

# MPC core library
add_library(mpc_core
  src/planner/local/mpc_motion_controller/mpc_core.cpp
)
target_include_directories(mpc_core PRIVATE ${CPPAD_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(mpc_core ipopt cppad_lib)  # 여기서 IPOPT, CppAD 라이브러리 링크 반드시!

# Main executable
add_executable(inabot_core ${SOURCES})

# Link dependencies
ament_target_dependencies(inabot_core
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs      # <- scan_merger용
  rcl_interfaces
  rcutils
  nav2_msgs
  yaml-cpp
  angles
  control_toolbox
  diagnostic_updater
  fmt
  visualization_msgs
  Eigen3
  pcl_conversions
  laser_geometry
  message_filters
  vision_msgs
  inabot_msgs
)

target_include_directories(inabot_core PRIVATE
  include
  include/kinematics
  include/motor
  include/utils
  include/monitor
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Link system and custom libraries
target_link_libraries(inabot_core
  mpc_core  # mpc_core 라이브러리 링크
  Boost::log
  Boost::log_setup
  Boost::thread
  Boost::system
  Boost::filesystem
  Boost::date_time
  Boost::stacktrace_backtrace
  ${EIGEN3_LIBRARIES}
  ${SERIAL_LIBRARY} # serial library
  control_toolbox::control_toolbox
  ${PCL_LIBRARIES}
)

# Install executable
install(TARGETS inabot_core
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and configuration files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
