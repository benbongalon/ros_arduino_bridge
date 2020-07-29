set(CMAKE_CXX_STANDARD 14)
cmake_minimum_required(VERSION 2.8.3)
project(ros_arduino_firmware)

# find dependencies
#find_package(ament_cmake REQUIRED)
#find_package(rclcpp REQUIRED)
#find_package(builtin_interfaces REQUIRED)

install(DIRECTORY src
  DESTINATION share
)
