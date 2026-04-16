cmake_minimum_required(VERSION 3.8)
project(natsurobo_early_access)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executable
add_executable(OmniDrive 
  src/zakiomni.cpp
)
target_include_directories(OmniDrive PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(OmniDrive PUBLIC c_std_99 cxx_std_17)

# Link dependencies
ament_target_dependencies(OmniDrive rclcpp std_msgs sensor_msgs geometry_msgs)

# Install
install(TARGETS OmniDrive
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()