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

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)

ament_target_dependencies(waypoint_navigator
  rclcpp
  nav2_msgs
  geometry_msgs
  yaml-cpp
  rclcpp_action
)
target_link_libraries(waypoint_navigator
  yaml-cpp
)

install(TARGETS
  waypoint_navigator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
