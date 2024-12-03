cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   # add_compile_options(-Wall -Wextra -Wpedantic)
#   add_compile_options(-O3 -Wall -Wextra -Wpedantic)
# endif()
# SET(CMAKE_CXX_FLAGS "-O2 -g ${CMAKE_CXX_FLAGS}") # こいつがbuildが終わらない原因？

# package.xmlのdependをチェックして自動的にfind_packageしてくれる

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_auto_find_build_dependencies()
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  # test
)


add_executable(map_matcher_node 
  src/map_matcher.cpp
)

target_link_libraries(map_matcher_node
  ${PCL_LIBRARIES}
  # ${rclcpp_lifecycle_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_LIBRARIES}
)
ament_target_dependencies(map_matcher_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  tf2_eigen
  # xmlrpcpp
)
install(TARGETS
  map_matcher_node
DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}/
)





add_executable(ekf_localizer_node 
  # src/tf_broadcaster_node.cpp
  # src/tf_broadcaster.cpp
  src/ekf.cpp
  # src/map_matcher.cpp
)
target_link_libraries(ekf_localizer_node
  ${PCL_LIBRARIES}
  # ${rclcpp_lifecycle_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# ament_auto_add_executable(ekf_localizer_node
#   src/tf_broadcaster.cpp
# )

# ament_package()
ament_target_dependencies(ekf_localizer_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  pcl_conversions
  tf2_eigen
  # xmlrpcpp
)

install(TARGETS
  ekf_localizer_node
DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}/
# )

add_executable(tf_cub_node 
  src/dynamic_tf_cub.cpp
)
target_link_libraries(tf_cub_node
  # ${PCL_LIBRARIES}
  # ${rclcpp_lifecycle_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

# ament_auto_add_executable(ekf_localizer_node
#   src/tf_broadcaster.cpp
# )

# ament_package()
ament_target_dependencies(tf_cub_node
  rclcpp
  std_msgs
  # sensor_msgs
  nav_msgs
  geometry_msgs
  # visualization_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  # pcl_conversions
  tf2_eigen
  # xmlrpcpp
)

install(TARGETS
  tf_cub_node
DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch 
#   DESTINATION share/${PROJECT_NAME}/
# )