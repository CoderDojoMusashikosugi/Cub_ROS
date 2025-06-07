cmake_minimum_required(VERSION 3.8)
project(perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters geometry)
find_package(pcl_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(VTK REQUIRED)
find_package(libusb REQUIRED)

# QHULLの設定
find_library(QHULL_LIBRARY qhull)
find_path(QHULL_INCLUDE_DIR libqhull/libqhull.h)

if(NOT QHULL_LIBRARY OR NOT QHULL_INCLUDE_DIR)
  message(FATAL_ERROR "QHULL library or headers not found")
endif()

# QHULLターゲットの作成
add_library(QHULL::QHULL UNKNOWN IMPORTED)
set_target_properties(QHULL::QHULL PROPERTIES
  IMPORTED_LOCATION "${QHULL_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${QHULL_INCLUDE_DIR}"
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(HumanTracker src/HumanTracker.cpp)
target_include_directories(HumanTracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(HumanTracker PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  HumanTracker
  "std_msgs"
  "sensor_msgs"
  "realsense2_camera_msgs"
  "cv_bridge"
  "image_transport"
  "pcl_ros"
  "visualization_msgs"
  "message_filters"
  "geometry_msgs"
  "tf2_ros"
  "tf2_geometry_msgs"
)

# PCLの依存関係を追加
target_link_libraries(HumanTracker ${PCL_LIBRARIES})
target_include_directories(HumanTracker PUBLIC ${PCL_INCLUDE_DIRS})
target_compile_definitions(HumanTracker PUBLIC ${PCL_DEFINITIONS})

# VTKの依存関係を追加
target_link_libraries(HumanTracker ${VTK_LIBRARIES})
target_include_directories(HumanTracker PUBLIC ${VTK_INCLUDE_DIRS})

# libusbの依存関係を追加
target_link_libraries(HumanTracker ${LIBUSB_LIBRARIES})
target_include_directories(HumanTracker PUBLIC ${LIBUSB_INCLUDE_DIRS})

# QHULLの依存関係を追加
target_link_libraries(HumanTracker QHULL::QHULL)

install(TARGETS HumanTracker
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
