cmake_minimum_required(VERSION 3.8)
project(plasma_robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PathTransformStatus.msg"
  "msg/PathExecutionStatus.msg"
  "msg/EntryMotionStatus.msg"
  "msg/SprayPath.msg"
  "msg/SprayPathPoint.msg"
  "srv/StartSprayPath.srv"
  "srv/TransformSprayPath.srv"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
