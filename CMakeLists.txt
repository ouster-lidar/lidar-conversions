cmake_minimum_required(VERSION 3.8)
project(lidar-conversions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(lidar_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# C++ library --------------------------------------------------------------
add_library(lidar_conversions_library SHARED
  src/lidar_scan_conversions.cpp
)

target_include_directories(lidar_conversions_library PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

ament_target_dependencies(lidar_conversions_library
  rclcpp
  lidar_msgs
  sensor_msgs
  pcl_conversions
)

install(TARGETS lidar_conversions_library
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(lidar_conversions_library)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(lidar_msgs sensor_msgs pcl_conversions)

# Python bindings ----------------------------------------------------------
# Builds lidar_conversions_py.so into this package's site-packages so rclpy
# nodes can `from lidar_conversions_py import lidar_scan_to_pointcloud2_data`.
find_package(pybind11_vendor QUIET)
find_package(pybind11 CONFIG REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

pybind11_add_module(lidar_conversions_py MODULE
  src/python_bindings.cpp
)

target_link_libraries(lidar_conversions_py PRIVATE
  lidar_conversions_library
)

ament_target_dependencies(lidar_conversions_py PUBLIC
  lidar_msgs
  sensor_msgs
)

# The .so sits at <prefix>/lib/python3.X/site-packages/lidar_conversions_py.so
# so it needs to find lidar_conversions_library.so in <prefix>/lib/.
set_target_properties(lidar_conversions_py PROPERTIES
  INSTALL_RPATH "$ORIGIN/../.."
)

install(TARGETS lidar_conversions_py
  LIBRARY DESTINATION
    "lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages"
)

# Tests --------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/lidar_scan_conversion_test.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    lidar_msgs
    sensor_msgs
    pcl_conversions
  )
  target_link_libraries(${PROJECT_NAME}_test lidar_conversions_library)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
