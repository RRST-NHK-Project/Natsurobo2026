cmake_minimum_required(VERSION 3.8)
project(manual_controll_natsu26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executable

add_executable(hardware_control_2
  src/mc_2026.cpp
)

target_include_directories(hardware_control_2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_control_2 PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(hardware_control_2 rclcpp std_msgs sensor_msgs geometry_msgs)

# Install
install(TARGETS hardware_control_2
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
