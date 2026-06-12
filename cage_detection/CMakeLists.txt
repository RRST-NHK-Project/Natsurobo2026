cmake_minimum_required(VERSION 3.8)
project(cage_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake             REQUIRED)
find_package(rclcpp                  REQUIRED)
find_package(std_msgs                REQUIRED)
find_package(geometry_msgs           REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cage.msg"
  "msg/CageArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(cage_detection_node
  src/cage_detection_node.cpp
)

target_compile_features(cage_detection_node PUBLIC cxx_std_17)

ament_target_dependencies(cage_detection_node
  rclcpp
  std_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(cage_detection_node "${cpp_typesupport_target}")

install(TARGETS cage_detection_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config msg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
