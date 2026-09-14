from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro
import yaml


def _load_config(package_share):
    config_path = os.path.join(package_share, "config", "tool_variants.yaml")
    with open(config_path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _vector(values):
    return " ".join(str(value) for value in values)


def _inertia_mappings(prefix, config):
    inertia = config["inertia"]
    return {
        f"{prefix}_mass": str(config["mass"]),
        f"{prefix}_inertial_xyz": _vector(config["inertial_xyz"]),
        f"{prefix}_inertial_rpy": _vector(config["inertial_rpy"]),
        f"{prefix}_ixx": str(inertia["ixx"]),
        f"{prefix}_ixy": str(inertia["ixy"]),
        f"{prefix}_ixz": str(inertia["ixz"]),
        f"{prefix}_iyy": str(inertia["iyy"]),
        f"{prefix}_iyz": str(inertia["iyz"]),
        f"{prefix}_izz": str(inertia["izz"]),
    }


def _launch_setup(context):
    package_share = get_package_share_directory("plasma_tool_description")
    config = _load_config(package_share)

    adapter_type = LaunchConfiguration("adapter_type").perform(context)
    spray_rod_type = LaunchConfiguration("spray_rod_type").perform(context)

    if adapter_type not in config["adapters"]:
        available = ", ".join(sorted(config["adapters"].keys()))
        raise RuntimeError(
            f"Unknown adapter_type '{adapter_type}'. Available: {available}"
        )
    rods = config["spray_rods"].get(adapter_type, {})
    if spray_rod_type not in rods:
        available = ", ".join(sorted(rods.keys()))
        raise RuntimeError(
            f"Unknown spray_rod_type '{spray_rod_type}' for adapter "
            f"'{adapter_type}'. Available: {available}"
        )

    assembly = config["assembly"]
    process_geometry = config["process_geometry"]
    adapter = config["adapters"][adapter_type]
    rod = rods[spray_rod_type]
    tip_to_outlet_center_m = (
        process_geometry["tip_to_outlet_near_edge_m"]
        + 0.5 * rod["outlet_axial_length_m"]
    )
    shaft_radius_m = process_geometry["shaft_outer_radius_m"][adapter_type]
    nozzle_tip_xyz = rod["tcp_xyz"]
    motion_tcp_xyz = [
        nozzle_tip_xyz[0] + shaft_radius_m,
        nozzle_tip_xyz[1] - tip_to_outlet_center_m,
        nozzle_tip_xyz[2],
    ]
    mappings = {
        "adapter_mesh": adapter["mesh"],
        "spray_rod_mesh": rod["mesh"],
        "mount_xyz": _vector(assembly["mount_xyz"]),
        "mount_rpy": _vector(assembly["mount_rpy"]),
        "adapter_xyz": _vector(assembly["adapter_xyz"]),
        "adapter_rpy": _vector(assembly["adapter_rpy"]),
        "rod_xyz": _vector(assembly["rod_xyz"]),
        "rod_rpy": _vector(assembly["rod_rpy"]),
        "nozzle_tip_xyz": _vector(nozzle_tip_xyz),
        "nozzle_tip_rpy": _vector(assembly["tcp_rpy"]),
        "motion_tcp_xyz": _vector(motion_tcp_xyz),
        "motion_tcp_rpy": _vector(process_geometry["motion_tcp_rpy_in_rod"]),
        "nozzle_tip_from_motion_tcp_xyz": _vector(
            [tip_to_outlet_center_m, 0.0, -shaft_radius_m]
        ),
        "spray_outlet_xyz": "0 0 0",
        "spray_outlet_rpy": "0 0 0",
    }
    mappings.update(_inertia_mappings("adapter", adapter))
    mappings.update(_inertia_mappings("spray_rod", rod))

    xacro_path = os.path.join(
        package_share, "urdf", "rm_eco65_with_plasma_tool.urdf.xacro"
    )
    robot_description = xacro.process_file(xacro_path, mappings=mappings).toxml()

    print(
        "[plasma_tool_description] selected "
        f"adapter={adapter_type}, spray_rod={spray_rod_type}, "
        f"nozzle_tip_xyz={nozzle_tip_xyz}, motion_tcp_xyz={motion_tcp_xyz}, "
        "spray_outlet_xyz=[0, 0, 0]"
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("use_joint_state_gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="plasma_tool_rviz",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            arguments=["-d", os.path.join(package_share, "rviz", "display.rviz")],
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("plasma_tool_description")
    defaults = _load_config(package_share)["defaults"]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "adapter_type", default_value=str(defaults["adapter_type"])
            ),
            DeclareLaunchArgument(
                "spray_rod_type", default_value=str(defaults["spray_rod_type"])
            ),
            DeclareLaunchArgument("use_joint_state_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
