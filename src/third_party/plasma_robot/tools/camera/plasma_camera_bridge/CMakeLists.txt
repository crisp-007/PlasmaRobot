cmake_minimum_required(VERSION 3.8)
project(plasma_camera_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d core)
find_package(rclcpp REQUIRED)
find_package(realsense2_camera_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(colored_pointcloud_node src/colored_pointcloud_node.cpp)
target_link_libraries(colored_pointcloud_node ${OpenCV_LIBS})
target_include_directories(colored_pointcloud_node PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(colored_pointcloud_node
  rclcpp
  realsense2_camera_msgs
  sensor_msgs)

install(TARGETS colored_pointcloud_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
