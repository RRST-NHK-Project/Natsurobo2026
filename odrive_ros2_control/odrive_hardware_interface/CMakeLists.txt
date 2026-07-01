cmake_minimum_required(VERSION 3.8)
project(odrive_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# C++17 is required for ROS2 Jazzy compatibility
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-implicit-fallthrough)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
# Explicitly find hardware_interface to ensure header paths are resolved
# regardless of the build environment (fixes build errors on fresh installs)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
ament_auto_find_build_dependencies()

include(FindPkgConfig)
pkg_search_module(LIBUSB1 REQUIRED libusb-1.0)

if(LIBUSB1_FOUND)
  include_directories(SYSTEM ${LIBUSB1_INCLUDE_DIRS})
else()
  message( FATAL_ERROR "Failed to find libusb-1.0" )
endif()

ament_auto_add_library(
  odrive_usb SHARED
  src/odrive_usb.cpp
)
target_link_libraries(
  odrive_usb
  ${LIBUSB1_LIBRARIES}
)

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/odrive_hardware_interface.cpp
)
target_link_libraries(
  ${PROJECT_NAME}
  odrive_usb
  hardware_interface::hardware_interface
)

pluginlib_export_plugin_description_file(hardware_interface odrive_hardware_interface.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
