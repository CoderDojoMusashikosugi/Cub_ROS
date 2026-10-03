cmake_minimum_required(VERSION 3.8)
project(cub4_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(spresense_imu_node src/spresense_imu_node.cpp)
ament_target_dependencies(spresense_imu_node
  rclcpp
  sensor_msgs
)
target_link_libraries(spresense_imu_node serial)

install(TARGETS
  spresense_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
