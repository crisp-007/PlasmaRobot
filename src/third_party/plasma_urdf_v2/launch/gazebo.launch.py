from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    plasma_share = get_package_share_directory("plasma_urdf_v2")
    gazebo_share = get_package_share_directory("gazebo_ros")
    xacro_path = PathJoinSubstitution([plasma_share, "urdf", "plasma_urdf_v2.xacro"])
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str,
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [gazebo_share, "/launch/gazebo.launch.py"]
                )
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tf_footprint_base",
                arguments=[
                    "--x",
                    "0",
                    "--y",
                    "0",
                    "--z",
                    "0",
                    "--roll",
                    "0",
                    "--pitch",
                    "0",
                    "--yaw",
                    "0",
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "base_footprint",
                ],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_plasma_urdf_v2",
                arguments=[
                    "-entity",
                    "plasma_urdf_v2",
                    "-topic",
                    "robot_description",
                ],
                output="screen",
            ),
        ]
    )
