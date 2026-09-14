cmake_minimum_required(VERSION 3.8)
project(plasma_tool_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes reference rviz urdf
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
