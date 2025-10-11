cmake_minimum_required(VERSION 3.8)
project(cub_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions)
find_package(pcl_ros)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(odom_to_tf src/odom_to_tf.cpp)
ament_target_dependencies(odom_to_tf rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
add_executable(odom_as_localization src/odom_as_localization.cpp)
ament_target_dependencies(odom_as_localization rclcpp nav_msgs geometry_msgs tf2 tf2_ros)
add_executable(clear_odom src/clear_odom.cpp)
ament_target_dependencies(clear_odom rclcpp std_msgs)

add_executable(tilt_correction_node src/tilt_correction_node.cpp)
ament_target_dependencies(tilt_correction_node rclcpp sensor_msgs message_filters pcl_conversions pcl_ros Eigen3)

install(TARGETS odom_to_tf odom_as_localization clear_odom
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  tilt_correction_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/distance_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
