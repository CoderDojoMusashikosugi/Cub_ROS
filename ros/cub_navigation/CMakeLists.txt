cmake_minimum_required(VERSION 3.8)
project(cub_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(
  DIRECTORY launch param
  DESTINATION share/${PROJECT_NAME}
)

add_executable(waypoint_manager src/waypoint_editor.cpp)
ament_target_dependencies(waypoint_manager rclcpp geometry_msgs visualization_msgs yaml-cpp)
install(TARGETS waypoint_manager DESTINATION lib/${PROJECT_NAME})
target_link_libraries(waypoint_manager
  yaml-cpp
)
add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)
ament_target_dependencies(waypoint_visualizer rclcpp geometry_msgs visualization_msgs yaml-cpp)
install(TARGETS waypoint_visualizer DESTINATION lib/${PROJECT_NAME})
target_link_libraries(waypoint_visualizer
  yaml-cpp
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
