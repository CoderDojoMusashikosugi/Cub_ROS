cmake_minimum_required(VERSION 3.0.2)
project(cub_bringup)

find_package(catkin REQUIRED)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  config
  src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
