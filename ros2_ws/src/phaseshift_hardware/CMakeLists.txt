cmake_minimum_required(VERSION 3.8)
project(phaseshift_hardware)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


add_library(phaseshift_hardware SHARED
  src/unity_diff_drive_hardware.cpp
)

ament_target_dependencies(phaseshift_hardware
  rclcpp
  hardware_interface
  pluginlib
  geometry_msgs
  nav_msgs
  tf2_ros
)

pluginlib_export_plugin_description_file(
  hardware_interface
  plugin.xml
)

install(
  TARGETS phaseshift_hardware
  DESTINATION lib
)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()