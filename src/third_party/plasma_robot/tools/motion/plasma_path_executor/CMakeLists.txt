cmake_minimum_required(VERSION 3.8)
project(plasma_path_executor)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(plasma_robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(path_executor_validation
  src/path_validation.cpp
  src/retreat_path.cpp
  src/reviewed_trajectory.cpp
  src/single_layer_path.cpp)
target_compile_features(path_executor_validation PUBLIC cxx_std_17)
target_include_directories(path_executor_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(path_executor_validation Eigen3::Eigen)
ament_target_dependencies(path_executor_validation
  geometry_msgs
  moveit_msgs
  plasma_robot_interfaces)

add_executable(path_executor_node src/path_executor_node.cpp)
target_compile_features(path_executor_node PUBLIC cxx_std_17)
target_link_libraries(path_executor_node path_executor_validation Eigen3::Eigen)
ament_target_dependencies(path_executor_node
  geometry_msgs
  plasma_robot_interfaces
  rclcpp
  rm_ros_interfaces
  std_msgs
  std_srvs)

add_executable(single_layer_commission_node src/single_layer_commission_node.cpp)
target_compile_features(single_layer_commission_node PUBLIC cxx_std_17)
target_link_libraries(single_layer_commission_node
  path_executor_validation
  Eigen3::Eigen
  yaml-cpp)
ament_target_dependencies(single_layer_commission_node
  ament_index_cpp
  geometry_msgs
  plasma_robot_interfaces
  rclcpp
  std_srvs)

add_executable(entry_motion_planner_node src/entry_motion_planner_node.cpp)
target_compile_features(entry_motion_planner_node PUBLIC cxx_std_17)
target_link_libraries(entry_motion_planner_node path_executor_validation Eigen3::Eigen)
ament_target_dependencies(entry_motion_planner_node
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  plasma_robot_interfaces
  rclcpp
  rm_ros_interfaces
  sensor_msgs
  std_srvs
  visualization_msgs)

if(BUILD_TESTING)
  ament_add_gtest(test_path_validation test/test_path_validation.cpp)
  if(TARGET test_path_validation)
    target_link_libraries(test_path_validation path_executor_validation)
    ament_target_dependencies(test_path_validation plasma_robot_interfaces)
  endif()
  ament_add_gtest(test_single_layer_path test/test_single_layer_path.cpp)
  if(TARGET test_single_layer_path)
    target_link_libraries(test_single_layer_path path_executor_validation Eigen3::Eigen)
    ament_target_dependencies(test_single_layer_path plasma_robot_interfaces)
  endif()
  ament_add_gtest(test_reviewed_trajectory test/test_reviewed_trajectory.cpp)
  if(TARGET test_reviewed_trajectory)
    target_link_libraries(test_reviewed_trajectory path_executor_validation)
    ament_target_dependencies(test_reviewed_trajectory moveit_msgs)
  endif()
  ament_add_gtest(test_retreat_path test/test_retreat_path.cpp)
  if(TARGET test_retreat_path)
    target_link_libraries(test_retreat_path path_executor_validation)
    ament_target_dependencies(test_retreat_path plasma_robot_interfaces)
  endif()
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS path_executor_validation path_executor_node single_layer_commission_node
  entry_motion_planner_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(path_executor_validation)
ament_export_dependencies(Eigen3 geometry_msgs moveit_msgs plasma_robot_interfaces)

ament_package()
