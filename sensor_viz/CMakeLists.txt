cmake_minimum_required(VERSION 3.8)
project(sensor_viz)

find_package(ament_cmake REQUIRED)
find_package(rclpy       REQUIRED)

install(PROGRAMS
  scripts/sensor_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_visualizer
)

install(PROGRAMS
  scripts/sensor_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sensor_supervisor
)

# 検知モジュールは .py のまま入れて sensor_supervisor から import 可能にする
install(PROGRAMS
  scripts/sensor_port_detect.py
  DESTINATION lib/${PROJECT_NAME}
)

# `ros2 run sensor_viz detect_ports` 用の CLI 名でも入れる
install(PROGRAMS
  scripts/sensor_port_detect.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME detect_ports
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
