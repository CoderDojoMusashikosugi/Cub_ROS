cmake_minimum_required(VERSION 3.8)
project(cub_behavior_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
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
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(
  include
  ${YAML_CPP_INCLUDE_DIRS}
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/AcquireFrontTarget.action"
  DEPENDENCIES geometry_msgs std_msgs
)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
add_library(bt_follow_zone_plugins SHARED src/follow_zone_plugins.cpp)
add_library(front_target_server_component SHARED src/front_target_server.cpp)
add_executable(front_target_server src/front_target_server_main.cpp)

add_dependencies(bt_follow_zone_plugins ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(front_target_server_component ${PROJECT_NAME}__rosidl_generator_cpp)
add_dependencies(front_target_server ${PROJECT_NAME}__rosidl_generator_cpp)

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
  std_msgs
)
ament_target_dependencies(front_target_server_component
  rclcpp
  rclcpp_action
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  rclcpp_components
)
ament_target_dependencies(front_target_server
  rclcpp
  rclcpp_components
)

rosidl_get_typesupport_target(cub_behavior_tree__rosidl_typesupport_cpp ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(bt_follow_zone_plugins ${cub_behavior_tree__rosidl_typesupport_cpp})
target_link_libraries(front_target_server_component ${cub_behavior_tree__rosidl_typesupport_cpp})
target_link_libraries(front_target_server ${cub_behavior_tree__rosidl_typesupport_cpp} front_target_server_component)
target_include_directories(bt_follow_zone_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
target_include_directories(front_target_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
target_include_directories(front_target_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(bt_follow_zone_plugins
  PRIVATE BT_PLUGIN_EXPORT)
target_compile_definitions(front_target_server_component PRIVATE FRONT_TARGET_SERVER_BUILDING_DLL)
target_compile_definitions(front_target_server PRIVATE FRONT_TARGET_SERVER_BUILDING_DLL)

rclcpp_components_register_nodes(front_target_server_component "cub_behavior_tree::FrontTargetServer")

pluginlib_export_plugin_description_file(nav2_behavior_tree bt_plugins.xml)

install(TARGETS waypoint_navigator
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS bt_follow_zone_plugins
  DESTINATION lib)
install(TARGETS front_target_server_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS front_target_server
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  bt_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include)

ament_export_dependencies(rosidl_default_runtime rclcpp_components)

ament_package()
