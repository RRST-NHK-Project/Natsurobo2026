cmake_minimum_required(VERSION 3.8)
project(natsu_senser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(imu_publisher src/imu_publisher.cpp)
target_compile_features(imu_publisher PUBLIC cxx_std_17)
ament_target_dependencies(imu_publisher rclcpp sensor_msgs std_msgs)

install(TARGETS imu_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
