from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    handeye_yaml = os.path.join(
        get_package_share_directory("plasma_eye_hand"),
        "config",
        "camera_to_gripper.yaml",
    )
    robot_urdf = os.path.join(
        get_package_share_directory("rm_description"),
        "urdf",
        "rm_eco65.urdf",
    )
    tcp_yaml = os.path.join(
        get_package_share_directory("plasma_tool_description"),
        "config",
        "tcp_3_4_200_1x10.yaml",
    )

    arguments = [
        DeclareLaunchArgument("handeye_yaml", default_value=handeye_yaml),
        DeclareLaunchArgument("tcp_yaml", default_value=tcp_yaml),
        DeclareLaunchArgument("source_refinement_yaml", default_value=""),
        DeclareLaunchArgument("camera_frame", default_value="camera_color_optical_frame"),
        DeclareLaunchArgument("base_frame", default_value="baselink"),
        DeclareLaunchArgument("gripper_frame", default_value="Link6"),
        DeclareLaunchArgument("tcp_frame", default_value="plasma_motion_tcp"),
        DeclareLaunchArgument("require_validated_handeye", default_value="true"),
        DeclareLaunchArgument("require_validated_tcp", default_value="true"),
        DeclareLaunchArgument("publish_unvalidated_preview", default_value="true"),
        DeclareLaunchArgument("cavity_axis_outward_compensation_m", default_value="0.0"),
        DeclareLaunchArgument("spray_axis_outward_compensation_m", default_value="0.0"),
    ]

    node = Node(
        package="plasma_path_transform",
        executable="path_transform_node",
        name="plasma_path_transform",
        output="screen",
        parameters=[{
            "handeye_yaml": LaunchConfiguration("handeye_yaml"),
            "tcp_yaml": LaunchConfiguration("tcp_yaml"),
            "source_refinement_yaml": LaunchConfiguration("source_refinement_yaml"),
            "camera_frame": LaunchConfiguration("camera_frame"),
            "base_frame": LaunchConfiguration("base_frame"),
            "gripper_frame": LaunchConfiguration("gripper_frame"),
            "tcp_frame": LaunchConfiguration("tcp_frame"),
            "robot_description_file": robot_urdf,
            "require_validated_handeye": ParameterValue(
                LaunchConfiguration("require_validated_handeye"), value_type=bool),
            "require_validated_tcp": ParameterValue(
                LaunchConfiguration("require_validated_tcp"), value_type=bool),
            "publish_unvalidated_preview": ParameterValue(
                LaunchConfiguration("publish_unvalidated_preview"), value_type=bool),
            "cavity_axis_outward_compensation_m": ParameterValue(
                LaunchConfiguration("cavity_axis_outward_compensation_m"), value_type=float),
            "spray_axis_outward_compensation_m": ParameterValue(
                LaunchConfiguration("spray_axis_outward_compensation_m"), value_type=float),
        }],
    )

    return LaunchDescription(arguments + [node])
