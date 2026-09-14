import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

    # End effector selection arguments
    tool_enabled_arg = DeclareLaunchArgument(
        "tool_enabled", default_value="false",
        description="Load plasma spray gun end effector"
    )
    use_printed_part_arg = DeclareLaunchArgument(
        "use_printed_part", default_value="false",
        description="Load 3D printed calibration tip"
    )
    adapter_variant_arg = DeclareLaunchArgument(
        "adapter_variant", default_value="3_4",
        description="Adapter variant: 3_4 or 6_8"
    )
    rod_variant_arg = DeclareLaunchArgument(
        "rod_variant", default_value="3_4_200_1x10",
        description="Rod variant, e.g. 3_4_150_1x5, 6_8_200_1x10"
    )

    rm_eco65_driver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_driver')),'launch', 'rm_eco65_driver.launch.py'))
    )

    rm_eco65_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_control')),'launch', 'rm_eco65_control.launch.py'))
    )

    rm_eco65_moveit_config = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('rm_eco65_config')),'launch', 'real_moveit_demo.launch.py')),
            launch_arguments={
                'tool_enabled': LaunchConfiguration('tool_enabled'),
                'use_printed_part': LaunchConfiguration('use_printed_part'),
                'adapter_variant': LaunchConfiguration('adapter_variant'),
                'rod_variant': LaunchConfiguration('rod_variant'),
            }.items()
    )

    return LaunchDescription([
    tool_enabled_arg,
    use_printed_part_arg,
    adapter_variant_arg,
    rod_variant_arg,
    rm_eco65_driver,
    rm_eco65_control,
    rm_eco65_moveit_config
    ])
