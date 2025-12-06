cmake_minimum_required(VERSION 3.8)
project(cub_diagnostics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(topic_hz_node
  src/topic_hz_node.cpp
)
ament_target_dependencies(topic_hz_node
  rclcpp
  diagnostic_msgs
  diagnostic_updater
)

add_executable(linux_state_node
  src/linux_state_node.cpp
)
ament_target_dependencies(linux_state_node
  rclcpp
  diagnostic_msgs
  diagnostic_updater
)

add_executable(jetson_state_node
  src/jetson_state_node.cpp
)
ament_target_dependencies(jetson_state_node
  rclcpp
  diagnostic_msgs
  diagnostic_updater
)

add_executable(diag_rviz_converter_node
  src/diag_rviz_converter_node.cpp
)
ament_target_dependencies(diag_rviz_converter_node
  rclcpp
  diagnostic_msgs
  rviz_2d_overlay_msgs
)

add_executable(fix_status_node
  src/fix_status_node.cpp
)
ament_target_dependencies(fix_status_node
  rclcpp
  diagnostic_msgs
  diagnostic_updater
  sensor_msgs
)

install(TARGETS topic_hz_node
  linux_state_node
  jetson_state_node
  diag_rviz_converter_node
  fix_status_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
