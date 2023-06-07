cmake_minimum_required(VERSION 3.5.0)
project(chaining_controller)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ros2_control REQUIRED)
find_package(controller_manager REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  ros2_control
)

install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(example_effort examples/example_effort.cpp)
ament_target_dependencies(example_effort
  rclcpp
  std_msgs
)

## Install
install(
  TARGETS
    example_effort

  DESTINATION
    lib/${PROJECT_NAME}
)

## effort_controller_upper
add_library(
  effort_controller_upper
  SHARED
  src/effort_controller.cpp
)
target_include_directories(
  effort_controller_upper
  PRIVATE
  include
)
ament_target_dependencies(
  effort_controller_upper
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface effort_controller_upper.xml)

install(
  TARGETS
  effort_controller_upper
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  effort_controller_upper
)

## chained_controller
add_library(
  chained_controller
  SHARED
  src/chained_controller.cpp
)
target_include_directories(
  chained_controller
  PRIVATE
  include
)
ament_target_dependencies(
  chained_controller
  controller_interface
  hardware_interface
  realtime_tools
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(controller_interface chained_controller.xml)

install(
  TARGETS
  chained_controller
  DESTINATION lib
)
install(
  DIRECTORY
    include
  DESTINATION include
)

ament_export_include_directories(
  include
)
ament_export_libraries(
  chained_controller
)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  interfaces
  lib
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
ament_package()
