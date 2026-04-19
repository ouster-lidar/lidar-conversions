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

include_directories(
  include
)

add_library(lidar_conversions_library SHARED
  src/lidar_scan_conversions.cpp
)

ament_target_dependencies(lidar_conversions_library
  rclcpp
  lidar_msgs
  sensor_msgs
  pcl_conversions
)

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
