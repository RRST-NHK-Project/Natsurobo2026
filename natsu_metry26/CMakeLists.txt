cmake_minimum_required(VERSION 3.8)
project(natsu_metry26)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Add executable
add_executable(guess_position_26
  src/summer2026_odometry.cpp
  src/matrix.cpp
)

target_include_directories(guess_position_26 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(guess_position_26 PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(guess_position_26 rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros)

# Install
install(TARGETS guess_position_26
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
