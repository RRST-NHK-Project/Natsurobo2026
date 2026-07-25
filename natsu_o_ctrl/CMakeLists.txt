cmake_minimum_required(VERSION 3.8)
project(natsu_o_ctrl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(natsu_o_ctrl
  src/natsu_o_ctrl.cpp
)

target_compile_features(natsu_o_ctrl PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(natsu_o_ctrl rclcpp std_msgs sensor_msgs)

# Install
install(TARGETS natsu_o_ctrl
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
