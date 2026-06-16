cmake_minimum_required(VERSION 3.8)
project(sensor_viz)

find_package(ament_cmake REQUIRED)
find_package(rclpy       REQUIRED)

install(PROGRAMS
  scripts/sensor_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_visualizer
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
