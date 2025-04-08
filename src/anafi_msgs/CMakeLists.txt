cmake_minimum_required(VERSION 3.5)
project(anafi_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Fast-RTPS is optional unless you're doing DDS tuning, but you can keep it if needed
# find_package(rosidl_typesupport_fastrtps_c REQUIRED)
# find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)

# Generate ROS interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoxData.msg"
  "msg/CollectCurrentState.msg"
  "msg/ControlData.msg"
  "msg/CurrentState.msg"
  "msg/DroneSize.msg"
  "msg/FrameYolo.msg"
  "msg/ImageKeyPoints.msg"
  "msg/KeyPoints.msg"
  "msg/KpYolo.msg"
  "msg/Matrices.msg"
  "msg/MovingReference.msg"
  "msg/Output.msg"
  "msg/PlotData.msg"
  "msg/PnPDataYolo.msg"
  "msg/Position.msg"
  "msg/Speed.msg"

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
