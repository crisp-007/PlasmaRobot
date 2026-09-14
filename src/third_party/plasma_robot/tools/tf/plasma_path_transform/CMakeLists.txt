cmake_minimum_required(VERSION 3.8)
project(plasma_path_transform)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(plasma_robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(path_transform_core src/path_transformer.cpp)
target_include_directories(path_transform_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(path_transform_core Eigen3::Eigen yaml-cpp)
ament_target_dependencies(path_transform_core geometry_msgs)

add_executable(path_transform_node src/path_transform_node.cpp)
target_link_libraries(path_transform_node path_transform_core)
ament_target_dependencies(path_transform_node
  geometry_msgs
  urdf
  kdl_parser
  orocos_kdl
  plasma_robot_interfaces
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_ros)

install(TARGETS path_transform_core path_transform_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_transformer test/test_path_transformer.cpp)
  if(TARGET test_path_transformer)
    target_link_libraries(test_path_transformer path_transform_core)
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(path_transform_core)
ament_package()
