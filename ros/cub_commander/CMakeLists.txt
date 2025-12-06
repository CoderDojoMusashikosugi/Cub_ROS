cmake_minimum_required(VERSION 3.8)
project(cub_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
)

add_executable(cub_commander_node src/cub_commander.cpp src/joypad.cpp src/joy_to_dualsense.cpp src/joy_to_xbox1914.cpp)
ament_target_dependencies(cub_commander_node rclcpp sensor_msgs geometry_msgs geometry_msgs std_msgs)

add_executable(nav2_action_client src/nav2_action_client.cpp)
ament_target_dependencies(nav2_action_client rclcpp rclcpp_action sensor_msgs geometry_msgs geometry_msgs nav2_msgs yaml-cpp ament_index_cpp)
target_link_libraries(nav2_action_client ${YAML_CPP_LIBRARIES})

add_executable(start_button_serial src/start_button_serial.cpp)
ament_target_dependencies(start_button_serial rclcpp std_msgs)
target_link_libraries(start_button_serial serial)

install(TARGETS cub_commander_node nav2_action_client start_button_serial
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY routes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
