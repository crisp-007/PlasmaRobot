from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    depth_profile = LaunchConfiguration("depth_module.profile")
    depth_preset = LaunchConfiguration("depth_module.visual_preset")
    rgb_profile = LaunchConfiguration("rgb_camera.profile")
    pointcloud_enabled = LaunchConfiguration("pointcloud__cuda_.enable")
    pointcloud_stream_filter = LaunchConfiguration("pointcloud__cuda_.stream_filter")
    align_depth = LaunchConfiguration("align_depth.enable")
    enable_sync = LaunchConfiguration("enable_sync")
    enable_confidence = LaunchConfiguration("enable_confidence")
    output_topic = LaunchConfiguration("bridge_output_topic")
    max_output_hz = LaunchConfiguration("bridge_max_output_hz")

    vendor_launch = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch", "rs_launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("camera_name", default_value="camera"),
        DeclareLaunchArgument("depth_module.profile", default_value="640,480,30"),
        DeclareLaunchArgument("depth_module.visual_preset", default_value="5"),
        DeclareLaunchArgument("rgb_camera.profile", default_value="640,480,30"),
        DeclareLaunchArgument("pointcloud__cuda_.enable", default_value="true"),
        DeclareLaunchArgument("pointcloud__cuda_.stream_filter", default_value="0"),
        DeclareLaunchArgument("align_depth.enable", default_value="false"),
        DeclareLaunchArgument("enable_sync", default_value="false"),
        DeclareLaunchArgument("enable_confidence", default_value="false"),
        DeclareLaunchArgument(
            "bridge_output_topic", default_value="/plasma/camera/colored_points"),
        DeclareLaunchArgument("bridge_max_output_hz", default_value="6.0"),
        GroupAction(
            actions=[
                # This Jetson build exposes the actual CUDA filter under this
                # name, while the vendor launch file still uses the old name.
                SetParameter(
                    name="pointcloud__cuda_.stream_filter",
                    value=ParameterValue(pointcloud_stream_filter, value_type=int),
                ),
                SetParameter(
                    name="pointcloud__cuda_.pointcloud_qos",
                    value="SENSOR_DATA",
                ),
                SetParameter(name="align__cuda_.enable", value=False),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(vendor_launch),
                    launch_arguments={
                        "camera_name": camera_name,
                        "depth_module.profile": depth_profile,
                        "depth_module.visual_preset": depth_preset,
                        "rgb_camera.profile": rgb_profile,
                        "pointcloud__cuda_.enable": pointcloud_enabled,
                        "align_depth.enable": align_depth,
                        "enable_sync": enable_sync,
                        "enable_confidence": enable_confidence,
                    }.items(),
                ),
            ],
        ),
        Node(
            package="plasma_camera_bridge",
            executable="colored_pointcloud_node",
            name="plasma_colored_pointcloud",
            output="screen",
            parameters=[{
                "output_topic": output_topic,
                "max_output_hz": ParameterValue(max_output_hz, value_type=float),
            }],
        ),
    ])
