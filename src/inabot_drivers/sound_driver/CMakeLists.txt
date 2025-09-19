cmake_minimum_required(VERSION 3.5)
project(sound_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  include/sound_driver      
)

add_executable(sound_node src/main.cpp src/sound.cpp)

ament_target_dependencies(sound_node rclcpp std_msgs)

install(TARGETS
  sound_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()