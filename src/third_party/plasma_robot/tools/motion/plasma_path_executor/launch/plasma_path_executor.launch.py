from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("speed_percent", default_value="5"),
        DeclareLaunchArgument("entry_approach_enabled", default_value="false"),
        DeclareLaunchArgument("collision_clearance_required", default_value="true"),
        DeclareLaunchArgument("required_collision_stage", default_value="8"),
        DeclareLaunchArgument("current_monitor_enabled", default_value="true"),
        DeclareLaunchArgument("current_delta_threshold_ma", default_value="1500.0"),
    ]

    node = Node(
        package="plasma_path_executor",
        executable="path_executor_node",
        name="plasma_path_executor",
        output="screen",
        parameters=[{
            "motion_enabled": ParameterValue(
                LaunchConfiguration("motion_enabled"), value_type=bool),
            "speed_percent": ParameterValue(
                LaunchConfiguration("speed_percent"), value_type=int),
            "entry_approach_enabled": ParameterValue(
                LaunchConfiguration("entry_approach_enabled"), value_type=bool),
            "collision_clearance_required": ParameterValue(
                LaunchConfiguration("collision_clearance_required"), value_type=bool),
            "required_collision_stage": ParameterValue(
                LaunchConfiguration("required_collision_stage"), value_type=int),
            "current_monitor_enabled": ParameterValue(
                LaunchConfiguration("current_monitor_enabled"), value_type=bool),
            "current_delta_threshold_ma": ParameterValue(
                LaunchConfiguration("current_delta_threshold_ma"), value_type=float),
        }],
    )
    return LaunchDescription(arguments + [node])
