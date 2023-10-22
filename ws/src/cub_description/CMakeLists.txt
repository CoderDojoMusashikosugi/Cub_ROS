cmake_minimum_required(VERSION 3.0.2)
project(cub_description)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS urdf xacro
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
  launch
  meshes
  urdf
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
