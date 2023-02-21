cmake_minimum_required(VERSION 3.8)
project(cfdp_wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcl_interfaces REQUIRED)

set(common_ros_depends
  rcl_interfaces
)

ament_export_dependencies(
  rclpy
  ${common_ros_depends}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  cfdp_wrapper/cfdp_wrapper.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config resource launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
