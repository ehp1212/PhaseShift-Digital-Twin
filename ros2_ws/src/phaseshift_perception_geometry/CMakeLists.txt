cmake_minimum_required(VERSION 3.8)
project(phaseshift_perception_geometry)

# -----------------------------
# C++ 
# -----------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# -----------------------------
# Dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(phaseshift_interfaces REQUIRED)

# -----------------------------
# Include directories
# -----------------------------
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# -----------------------------
# Executable 
# -----------------------------
add_executable(voxel_perception_node
  src/voxel_perception_node.cpp
  src/voxel_perception_main.cpp
)

# -----------------------------
# Target include 
# -----------------------------
target_include_directories(voxel_perception_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# -----------------------------
# Link libraries
# -----------------------------
target_link_libraries(voxel_perception_node
  ${PCL_LIBRARIES}
)

# -----------------------------
# Dependencies (ROS)
# -----------------------------
ament_target_dependencies(voxel_perception_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  geometry_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  phaseshift_interfaces
)

# -----------------------------
# Install
# -----------------------------
install(TARGETS
  voxel_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  phaseshift_perception_geometry/voxel_change_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  phaseshift_perception_geometry/voxel_costmap_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# -----------------------------
# Ament
# -----------------------------
ament_package()