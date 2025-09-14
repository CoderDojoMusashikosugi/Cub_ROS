cmake_minimum_required(VERSION 3.8)
project(handy1_bringup)

find_package(ament_cmake REQUIRED)

add_executable(pwm_timesync src/pwm_timesync.cpp)
target_link_libraries(pwm_timesync pigpio pthread)

install(TARGETS pwm_timesync
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY web
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/web_control_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
