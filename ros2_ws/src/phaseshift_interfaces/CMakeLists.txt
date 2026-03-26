cmake_minimum_required(VERSION 3.8)
project(phaseshift_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CostmapGrid.msg"
  "msg/NavigationFeedback.msg"
  "msg/SystemState.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"

  "msg/internal/NavState.msg"

  "srv/SaveMap.srv"
  "srv/SetGoal.srv"
  "srv/NavigateToGoal.srv"

  "srv/internal/InternalCancelNavigate.srv"
  "srv/internal/InternalRequestNavigate.srv"
  "srv/internal/InternalRequestRecovery.srv"

  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()