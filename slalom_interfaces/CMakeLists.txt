cmake_minimum_required(VERSION 3.5)
project(slalom_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/State.msg"
  "msg/Setpoint.msg"
  "msg/CameraDetection.msg"
  "msg/CameraDetections.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
