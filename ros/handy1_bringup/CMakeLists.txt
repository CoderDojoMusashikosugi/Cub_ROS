cmake_minimum_required(VERSION 3.8)
project(handy1_bringup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pwm_timesync src/pwm_timesync.cpp)
target_link_libraries(pwm_timesync pigpio pthread)

# Frequency Checker Component
add_library(frequency_checker SHARED src/frequency_checker.cpp)
target_compile_features(frequency_checker PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(frequency_checker
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(frequency_checker
  PLUGIN "handy1_bringup::FrequencyChecker"
  EXECUTABLE frequency_checker_node
)

install(TARGETS pwm_timesync
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS frequency_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/web_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
