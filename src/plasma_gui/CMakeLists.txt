cmake_minimum_required(VERSION 3.16)

project(plasma_gui VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets SerialPort)#OpenGL
#set(Qt6_DIR "/home/larusxu/Qt/6.7.2/gcc_arm64" CACHE PATH "Select Qt6 version")
list(PREPEND CMAKE_PREFIX_PATH "/home/larusxu/Qt/6.7.2/gcc_arm64")
find_package(Qt6 6.7.2 REQUIRED COMPONENTS Widgets LinguistTools OpenGLWidgets SerialPort)
#find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets LinguistTools OpenGL OpenGLWidgets SerialPort)

#ROS2相关
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rm_ros_interfaces REQUIRED)
find_package(plasma_robot_interfaces REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

#导入Open3D
# 设置Open3D的查找路径
#set(Open3D_DIR "/usr/local/lib/cmake/Open3D")  # Open3D提供的CMake配置路径
#list(APPEND CMAKE_PREFIX_PATH "${Open3D_DIR}")
message(STATUS "============PointCloud============")
message(STATUS "++++++Open3D++++++")
# 查找Open3D包
find_package(Open3D REQUIRED)
# 检查是否找到Open3D
if(Open3D_FOUND)
    message(STATUS "✅Open3D found! Version: ${Open3D_VERSION}")
else()
    message(WARNING "Open3D not found!")
endif()

message(STATUS "++++++PCL++++++")
#导入PCL
#set(PCL_DIR "/usr/local/PCL1.15.0/share/pcl-1.15")
find_package(PCL REQUIRED)
if(PCL_FOUND)
    message(STATUS "✅PCL found! Version: ${PCL_VERSION}")
else()
    message(WARNING "PCL not found!")
endif()
message(STATUS "============PointCloud End============")
#导入VTK
message(STATUS "++++++VTK++++++")
#set(VTK_DIR "/usr/local/VTK9.3.0/lib/cmake/vtk-9.3")
find_package(VTK REQUIRED)
if (VTK_FOUND)
    message(STATUS "✅VTK found! Version: ${VTK_VERSION}")
    #message(STATUS "VTK include dirs = ${VTK_INCLUDE_DIRS}")
    message(STATUS "VTK libraries = ${VTK_LIBRARIES}")
else()
    message(FATAL_ERROR "VTK not found!")
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/runtime_libs.sh.in")

#头文件路径
set(include_dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/src/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/third_party
    
    ${CMAKE_CURRENT_SOURCE_DIR}/src/module/PointCloud
    ${CMAKE_CURRENT_SOURCE_DIR}/src/module/RobotArm

    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/mainwindow
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/statusbar/state_btn
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/serial
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/work_space/opengl
    # ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/work_space/opengl/point_v1
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/work_space/opengl/point_v2
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/dealtime
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/gas_state
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/param_set
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/patient
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/dbtree
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/step_wizard
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/step_wizard/ros_launch_manager
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/step_wizard/step/system_check
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/panel/left_panel/log
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/tip_ctl
    ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/ros_launch_manager
)

#IDE中显示头文件/UI文件
file(GLOB_RECURSE HEAD_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.h)
list(FILTER HEAD_FILE EXCLUDE REGEX ".*/build/.*|.*/bin/.*")
list(FILTER HEAD_FILE EXCLUDE REGEX ".*/point_v1/.*")
list(FILTER HEAD_FILE EXCLUDE REGEX ".*/demo_rotation_executor\\.h$")
file(GLOB_RECURSE UI_FILE ${CMAKE_CURRENT_SOURCE_DIR}/src/*.ui)
set(FILE_DISPLAY ${HEAD_FILE} ${UI_FILE})

set(TS_FILES PlasmaRobot_zh_CN.ts)

#指定源文件
file(GLOB_RECURSE PROJECT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
list(FILTER PROJECT_SOURCES EXCLUDE REGEX "point_v1")
list(FILTER PROJECT_SOURCES EXCLUDE REGEX ".*/demo_rotation_executor\\.cpp$")

set(QT_VERSION_MAJOR 6)
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    add_executable(plasma_gui
        #MANUAL_FINALIZATION
        ${FILE_DISPLAY}
        ${PROJECT_SOURCES}
        src/gui/resources.qrc
    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET PlasmaRobot APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation

    qt_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
else()
    if(ANDROID)
        add_library(PlasmaRobot SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(plasma_gui
            ${PROJECT_SOURCES}
            ${FILE_DISPLAY}
        )
    endif()

    qt5_create_translation(QM_FILES ${CMAKE_SOURCE_DIR} ${TS_FILES})
endif()

ament_target_dependencies(
  plasma_gui
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "std_srvs"
  "rm_ros_interfaces"
  "plasma_robot_interfaces"
  "ament_index_cpp"
  "yaml_cpp_vendor"
)

target_compile_definitions(plasma_gui PRIVATE
    PLASMA_GUI_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)

target_link_libraries(plasma_gui 
                            Qt${QT_VERSION_MAJOR}::Widgets
                            Qt${QT_VERSION_MAJOR}::OpenGLWidgets
                            Qt${QT_VERSION_MAJOR}::SerialPort
                            ${PCL_LIBRARIES}
                            ${VTK_LIBRARIES}
                            qhull_r)
target_include_directories(plasma_gui 
                            PUBLIC
                            ${include_dirs}
                            PRIVATE 
                            ${PCL_INCLUDE_DIRS}
                            ${VTK_INCLUDE_DIRS}
)

# 在Windows平台添加dwmapi库支持暗色标题栏
if(WIN32)
    target_link_libraries(plasma_gui PRIVATE dwmapi)
endif()

if(BUILD_TESTING)
    enable_testing()
    add_executable(plasma_controller_protocol_test
        test/plasma_controller_protocol_test.cpp
        src/gui/serial/plasma_controller_protocol.cpp
    )
    target_include_directories(plasma_controller_protocol_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/gui/serial
    )
    target_link_libraries(plasma_controller_protocol_test PRIVATE Qt6::Core)
    add_test(NAME plasma_controller_protocol_test COMMAND plasma_controller_protocol_test)
endif()

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
#if(${QT_VERSION} VERSION_LESS 6.1.0)
#  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.plasma_gui)
#endif()
#set_target_properties(plasma_gui PROPERTIES
#    ${BUNDLE_ID_OPTION}
#    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#    MACOSX_BUNDLE TRUE
#   WIN32_EXECUTABLE TRUE
#)

include(GNUInstallDirs)
install(TARGETS plasma_gui
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/mock_arm_joint_states.py
    scripts/launch_plasma_gui.sh
    DESTINATION lib/${PROJECT_NAME}
)

# 安装 .desktop 文件（配置路径后）
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/plasma_gui.desktop.in
    ${CMAKE_CURRENT_BINARY_DIR}/plasma_gui.desktop
    @ONLY
)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/plasma_gui.desktop
    DESTINATION share/applications
)

# 安装应用图标
install(FILES src/gui/ui_source/winicon.png
    DESTINATION share/${PROJECT_NAME}/icons
)

#if(QT_VERSION_MAJOR EQUAL 6)
#    qt_finalize_executable(plasma_gui)
#endif()


ament_package()
