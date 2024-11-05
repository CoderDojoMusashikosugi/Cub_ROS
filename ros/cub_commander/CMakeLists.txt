cmake_minimum_required(VERSION 3.8)
project(cub_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(cub_commander_node src/cub_commander.cpp src/joy_to_dualsense.cpp)
ament_target_dependencies(cub_commander_node rclcpp sensor_msgs geometry_msgs geometry_msgs)

install(TARGETS cub_commander_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
