cmake_minimum_required(VERSION 3.8)
project(cub_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(
  include
)

add_executable(teleop_joy src/teleop_joy.cpp src/joy_to_dualsense.cpp)
ament_target_dependencies(teleop_joy rclcpp sensor_msgs geometry_msgs geometry_msgs)

install(TARGETS teleop_joy
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
