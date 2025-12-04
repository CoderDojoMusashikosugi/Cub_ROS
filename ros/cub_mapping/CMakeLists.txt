cmake_minimum_required(VERSION 3.8)
project(cub_mapping)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Qt5 (required for RViz plugins)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# PCL
find_package(PCL REQUIRED COMPONENTS common filters io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddRegion.srv"
  "srv/AddWaypoint.srv"
  "srv/DeleteWaypoint.srv"
  "srv/MoveWaypoint.srv"
  "srv/UpdateWaypointYaw.srv"
  "srv/InsertWaypoint.srv"
  "srv/CreateGroup.srv"
  "srv/DeleteGroup.srv"
  "srv/RenameGroup.srv"
  "srv/SetRequireInput.srv"
  "srv/LoadWaypoints.srv"
  "srv/GetGroupList.srv"
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Include directories
include_directories(
  include
  ${Qt5Widgets_INCLUDE_DIRS}
)

# RViz Tool Plugin Library
add_library(${PROJECT_NAME}_plugin SHARED
  src/pointcloud_to_2d_tool.cpp
  include/cub_mapping/pointcloud_to_2d_tool.hpp
  src/waypoint_edit_tool.cpp
  include/cub_mapping/waypoint_edit_tool.hpp
)

set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
  AUTOMOC ON
)

target_include_directories(${PROJECT_NAME}_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}_plugin PRIVATE "CUB_MAPPING_BUILDING_LIBRARY")

ament_target_dependencies(${PROJECT_NAME}_plugin
  rclcpp
  std_msgs
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  visualization_msgs
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_plugin
  Qt5::Widgets
)

# Ensure service interface is available
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_plugin "${cpp_typesupport_target}")

# Map Converter Node
add_executable(map_converter_node
  src/map_converter_node.cpp
)

target_include_directories(map_converter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(map_converter_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  pcl_ros
)

target_link_libraries(map_converter_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  "${cpp_typesupport_target}"
)

# Terrain Analysis Node
add_executable(terrain_analysis_node
  src/terrain_analysis.cpp
)

target_include_directories(terrain_analysis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(terrain_analysis_node
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  geometry_msgs
  pcl_conversions
  pcl_ros
)

target_link_libraries(terrain_analysis_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# Waypoint Manager Node
add_executable(waypoint_manager_node
  src/waypoint_manager_node.cpp
)

target_include_directories(waypoint_manager_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(waypoint_manager_node
  rclcpp
  std_msgs
  std_srvs
  visualization_msgs
  geometry_msgs
)

target_link_libraries(waypoint_manager_node
  yaml-cpp
  "${cpp_typesupport_target}"
)

# Install node executables
install(TARGETS
  map_converter_node
  terrain_analysis_node
  waypoint_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install plugin library
install(TARGETS ${PROJECT_NAME}_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# Install launch files, params, and rviz config
install(DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_plugin)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_rendering
  rviz_ogre_vendor
  pluginlib
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  rosidl_default_runtime
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
