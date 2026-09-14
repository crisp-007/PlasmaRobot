from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    tcp_yaml = os.path.join(
        get_package_share_directory("plasma_tool_description"),
        "config",
        "tcp_3_4_200_1x10.yaml",
    )
    arguments = [
        DeclareLaunchArgument("commissioning_enabled", default_value="false"),
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("speed_percent", default_value="5"),
        DeclareLaunchArgument("angular_step_deg", default_value="5.0"),
        DeclareLaunchArgument("virtual_spray_distance_m", default_value="0.05"),
        DeclareLaunchArgument("layer_count", default_value="2"),
        DeclareLaunchArgument("layer_step_m", default_value="0.010"),
        DeclareLaunchArgument("tcp_yaml", default_value=tcp_yaml),
    ]

    commission_node = Node(
        package="plasma_path_executor",
        executable="single_layer_commission_node",
        name="plasma_multi_layer_commission",
        output="screen",
        parameters=[{
            "commissioning_enabled": ParameterValue(
                LaunchConfiguration("commissioning_enabled"), value_type=bool),
            "angular_step_deg": ParameterValue(
                LaunchConfiguration("angular_step_deg"), value_type=float),
            "virtual_spray_distance_m": ParameterValue(
                LaunchConfiguration("virtual_spray_distance_m"), value_type=float),
            "layer_count": ParameterValue(
                LaunchConfiguration("layer_count"), value_type=int),
            "layer_step_m": ParameterValue(
                LaunchConfiguration("layer_step_m"), value_type=float),
            "tcp_yaml": LaunchConfiguration("tcp_yaml"),
        }],
    )
    executor_node = Node(
        package="plasma_path_executor",
        executable="path_executor_node",
        name="plasma_path_executor",
        output="screen",
        parameters=[{
            "motion_enabled": ParameterValue(
                LaunchConfiguration("motion_enabled"), value_type=bool),
            "speed_percent": ParameterValue(
                LaunchConfiguration("speed_percent"), value_type=int),
        }],
    )
    return LaunchDescription(arguments + [commission_node, executor_node])
