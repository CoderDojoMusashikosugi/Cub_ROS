cmake_minimum_required(VERSION 3.8)
project(cub_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
add_library(bt_follow_zone_plugins SHARED src/follow_zone_plugins.cpp)

ament_target_dependencies(waypoint_navigator
  rclcpp
  nav2_msgs
  geometry_msgs
  yaml-cpp
  rclcpp_action
  std_msgs
)
target_link_libraries(waypoint_navigator
  yaml-cpp
)

ament_target_dependencies(bt_follow_zone_plugins
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  pluginlib
  nav2_behavior_tree
)

pluginlib_export_plugin_description_file(nav2_behavior_tree bt_plugins.xml)

install(TARGETS waypoint_navigator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bt_follow_zone_plugins
  DESTINATION lib)

install(FILES
  bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
