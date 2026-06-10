cmake_minimum_required(VERSION 3.8)
project(shooting_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake             REQUIRED)
find_package(rclcpp                  REQUIRED)
find_package(std_msgs                REQUIRED)
find_package(geometry_msgs           REQUIRED)
find_package(cage_detection          REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── カスタムメッセージ ──────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShooterCommand.msg"
  DEPENDENCIES std_msgs
)

# ── ノード ─────────────────────────────────────
add_executable(shooting_control_node
  src/shooting_control_node.cpp
)

target_compile_features(shooting_control_node PUBLIC cxx_std_17)

ament_target_dependencies(shooting_control_node
  rclcpp
  std_msgs
  geometry_msgs
  cage_detection
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(shooting_control_node "${cpp_typesupport_target}")

install(TARGETS shooting_control_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
