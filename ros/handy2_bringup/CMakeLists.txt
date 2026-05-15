cmake_minimum_required(VERSION 3.8)
project(handy2_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pico_node src/pico.cpp src/pico_comm.cpp)
ament_target_dependencies(pico_node rclcpp sensor_msgs)
install(TARGETS pico_node DESTINATION lib/${PROJECT_NAME})

# Include directories
include_directories(
  include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/web_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
