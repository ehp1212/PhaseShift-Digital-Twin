cmake_minimum_required(VERSION 3.8)
project(phaseshift_perception_geometry)

# -----------------------------
# dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(rclcpp_lifecycle REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(phaseshift_interfaces REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

# -----------------------------
# Python install
# -----------------------------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  phaseshift_perception_geometry/voxel_change_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  # RENAME voxel_change_detection_node  
)

install(PROGRAMS
  phaseshift_perception_geometry/voxel_costmap_node.py
  DESTINATION lib/${PROJECT_NAME}
  # RENAME voxel_costmap_node
)

# -----------------------------
# C++ node 
# -----------------------------
add_executable(voxel_perception_node
  src/voxel_perception_node.cpp
)

target_link_libraries(voxel_perception_node
  ${PCL_LIBRARIES}
)

ament_target_dependencies(voxel_perception_node
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  pcl_conversions
  tf2_ros
  tf2_sensor_msgs
  phaseshift_interfaces
)

install(TARGETS
  voxel_perception_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


# -----------------------------
ament_package()