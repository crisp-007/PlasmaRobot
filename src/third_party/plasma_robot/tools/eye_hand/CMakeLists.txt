cmake_minimum_required(VERSION 3.8)
project(plasma_eye_hand)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(plasma_robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(tcp_pivot_core src/tcp_pivot.cpp)
target_compile_features(tcp_pivot_core PUBLIC cxx_std_17)
target_include_directories(tcp_pivot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(tcp_pivot_core PUBLIC Eigen3::Eigen)

add_library(reach_validation_core src/reach_validation.cpp)
target_compile_features(reach_validation_core PUBLIC cxx_std_17)
target_include_directories(reach_validation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(reach_validation_core PUBLIC Eigen3::Eigen)

add_library(reach_refinement_core src/reach_refinement.cpp)
target_compile_features(reach_refinement_core PUBLIC cxx_std_17)
target_include_directories(reach_refinement_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(reach_refinement_core PUBLIC Eigen3::Eigen)

add_library(depth_alignment_core src/depth_alignment.cpp)
target_compile_features(depth_alignment_core PUBLIC cxx_std_17)
target_include_directories(depth_alignment_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(depth_alignment_core PUBLIC Eigen3::Eigen)

add_executable(tcp_pivot_calibrator src/tcp_pivot_calibrator.cpp)
target_compile_features(tcp_pivot_calibrator PUBLIC cxx_std_17)
target_link_libraries(tcp_pivot_calibrator tcp_pivot_core yaml-cpp)
ament_target_dependencies(tcp_pivot_calibrator
  ament_index_cpp
  geometry_msgs
  rclcpp)

add_executable(reach_validation_recorder src/reach_validation_recorder.cpp)
target_compile_features(reach_validation_recorder PUBLIC cxx_std_17)
target_link_libraries(reach_validation_recorder reach_validation_core yaml-cpp)
ament_target_dependencies(reach_validation_recorder
  ament_index_cpp
  geometry_msgs
  plasma_robot_interfaces
  rclcpp)

add_executable(reach_refinement_analyzer src/reach_refinement_analyzer.cpp)
target_compile_features(reach_refinement_analyzer PUBLIC cxx_std_17)
target_link_libraries(reach_refinement_analyzer reach_refinement_core yaml-cpp)

add_executable(robust_handeye_analyzer src/robust_handeye_analyzer.cpp)
target_compile_features(robust_handeye_analyzer PUBLIC cxx_std_17)
target_link_libraries(robust_handeye_analyzer Eigen3::Eigen yaml-cpp)

add_executable(depth_alignment_analyzer src/depth_alignment_analyzer.cpp)
target_compile_features(depth_alignment_analyzer PUBLIC cxx_std_17)
target_link_libraries(depth_alignment_analyzer depth_alignment_core yaml-cpp)

if(BUILD_TESTING)
  ament_add_gtest(test_tcp_pivot test/test_tcp_pivot.cpp)
  if(TARGET test_tcp_pivot)
    target_link_libraries(test_tcp_pivot tcp_pivot_core)
  endif()
  ament_add_gtest(test_reach_validation test/test_reach_validation.cpp)
  if(TARGET test_reach_validation)
    target_link_libraries(test_reach_validation reach_validation_core)
  endif()
  ament_add_gtest(test_reach_refinement test/test_reach_refinement.cpp)
  if(TARGET test_reach_refinement)
    target_link_libraries(test_reach_refinement reach_refinement_core)
  endif()
  ament_add_gtest(test_depth_alignment test/test_depth_alignment.cpp)
  if(TARGET test_depth_alignment)
    target_link_libraries(test_depth_alignment depth_alignment_core)
  endif()
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md ENTRY_REACH_VALIDATION_20260725.md
  DEPTH_ALIGNMENT_CALIBRATION_20260727.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS
  scripts/collect_depth_charuco_ros.py
  scripts/export_charuco_samples.py
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS tcp_pivot_core tcp_pivot_calibrator reach_validation_core reach_validation_recorder
  reach_refinement_core reach_refinement_analyzer robust_handeye_analyzer
  depth_alignment_core depth_alignment_analyzer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(tcp_pivot_core reach_validation_core reach_refinement_core depth_alignment_core)
ament_export_dependencies(Eigen3)

ament_package()
