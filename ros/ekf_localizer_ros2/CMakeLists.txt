cmake_minimum_required(VERSION 3.5)
project(ekf_localizer)

# add_compile_options(-std=c++14 -Wall)
# add_compile_options(-O2)

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
find_package(ndt_omp REQUIRED)
find_package(fast_gicp REQUIRED)
# find_package(pclomp REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(GeographicLib REQUIRED)

# find_package(PkgConfig REQUIRED)
# pkg_check_modules(GeographicLib REQUIRED geographic)

set(GeographicLib_INCLUDE_DIRS "/usr/include")
set(GeographicLib_LIBRARIES "Geographic")

ament_auto_find_build_dependencies()


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(OpenMP)
if(OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${ndt_omp_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${fast_gicp_INCLUDE_DIRS}
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
  # pclomp
  ndt_omp
  # xmlrpcpp
  fast_gicp
)

target_link_libraries(map_matcher_node
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_LIBRARIES}
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

# GPS Updater Node
add_executable(gps_updater_node 
  src/gps_updater.cpp
)

target_link_libraries(gps_updater_node
  ${GeographicLib_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

ament_target_dependencies(gps_updater_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  gps_updater_node
DESTINATION lib/${PROJECT_NAME})



# add_executable(pcd_renamer src/pcd_rename.cpp)

# ament_target_dependencies(pcd_renamer
#   rclcpp
#   sensor_msgs
# )

# install(TARGETS
#   pcd_renamer
#   DESTINATION lib/${PROJECT_NAME}
# )

# add_executable(draw_moving_trajectory_node src/draw_moving_trajectory.cpp)

# # Specify libraries to link a library or executable target against
# ament_target_dependencies(draw_moving_trajectory_node
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   std_msgs
#   visualization_msgs
#   std_srvs
# )

# # Install executables
# install(TARGETS
#   draw_moving_trajectory_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
