cmake_minimum_required(VERSION 3.8)
project(control_mcub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)

include_directories(include)
################################################################################
# mcub Control Mode
################################################################################
add_executable(control_mcub_moter_node src/control_mcub_moter_node.cpp)
ament_target_dependencies(control_mcub_moter_node
  dynamixel_sdk
  dynamixel_sdk_custom_interfaces
  geometry_msgs
  rclcpp
)
add_executable(mcub_teleop_key src/mcub_teleop_key.cpp)
ament_target_dependencies(mcub_teleop_key
  dynamixel_sdk
  geometry_msgs
  rclcpp
)
################################################################################
# Install
################################################################################
install(TARGETS
  control_mcub_moter_node
  mcub_teleop_key
  DESTINATION lib/${PROJECT_NAME}
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
