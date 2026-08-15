cmake_minimum_required(VERSION 3.8)
project(natsu_auto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(nav_msgs      REQUIRED)

add_executable(natsu_auto_node src/natsu_auto_node.cpp)
target_compile_features(natsu_auto_node PUBLIC cxx_std_17)
ament_target_dependencies(natsu_auto_node rclcpp std_msgs geometry_msgs)

install(TARGETS natsu_auto_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
