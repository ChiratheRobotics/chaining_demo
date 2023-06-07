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
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

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

ament_package()
