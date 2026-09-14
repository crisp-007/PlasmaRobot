cmake_minimum_required(VERSION 3.8)

project(plasma_urdf_v2)

find_package(ament_cmake REQUIRED)

foreach(dir config launch meshes rviz textures urdf)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(
      DIRECTORY ${dir}/
      DESTINATION share/${PROJECT_NAME}/${dir}
    )
  endif()
endforeach()

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()
