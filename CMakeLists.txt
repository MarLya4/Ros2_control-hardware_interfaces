cmake_minimum_required(VERSION 3.8)
project(example_hardware_interface)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

include_directories(
  include
  /usr/include
)

add_library(example_system SHARED
  src/example_system.cpp
)

ament_target_dependencies(example_system
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)


pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

install(
  TARGETS example_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
