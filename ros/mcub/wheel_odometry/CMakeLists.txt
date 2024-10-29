cmake_minimum_required(VERSION 3.8)
project(wheel_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 環境変数 CUB_TARGET を取得
if(DEFINED ENV{CUB_TARGET})
    message(STATUS "CUB_TARGET is set to $ENV{CUB_TARGET}")

    if($ENV{CUB_TARGET} STREQUAL "cub2")
        add_definitions(-DCUB_TARGET_CUB2)
    elseif($ENV{CUB_TARGET} STREQUAL "mcub")
        add_definitions(-DCUB_TARGET_MCUB)
    else()
        message(WARNING "CUB_TARGET is set to an unknown value: $ENV{CUB_TARGET}")
    endif()
else()
    message(FATAL_ERROR "CUB_TARGET environment variable is not set")
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(wheel_odometry_node src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odometry_node rclcpp std_msgs nav_msgs tf2 tf2_ros)

install(TARGETS
  wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME})


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
