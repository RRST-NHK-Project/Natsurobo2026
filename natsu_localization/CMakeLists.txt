cmake_minimum_required(VERSION 3.8)
project(natsu_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(tf2           REQUIRED)
find_package(tf2_ros       REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3        REQUIRED)

add_executable(scan_matcher_localizer
  src/scan_matcher_localizer.cpp
)

target_compile_features(scan_matcher_localizer PUBLIC cxx_std_17)

target_link_libraries(scan_matcher_localizer Eigen3::Eigen)

ament_target_dependencies(scan_matcher_localizer
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS scan_matcher_localizer
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
