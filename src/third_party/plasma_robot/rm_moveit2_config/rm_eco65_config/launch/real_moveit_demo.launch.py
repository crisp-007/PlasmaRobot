import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import xacro


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def build_tool_urdf(adapter_variant, rod_variant):
    """Generate URDF XML for the plasma spray gun with given adapter/rod variants."""
    # Read variant data from YAML
    variants = load_yaml("plasma_urdf_v2", "config/tool_variants.yaml")
    if not variants:
        return ""

    # Strip adapter prefix from rod variant key (e.g. "3_4_150_1x5" -> "150_1x5")
    rod_key = rod_variant
    prefix = adapter_variant + "_"
    if rod_key.startswith(prefix):
        rod_key = rod_key[len(prefix):]

    adapter = variants["adapters"].get(adapter_variant)
    rod = variants["spray_rods"].get(adapter_variant, {}).get(rod_key)
    if not adapter or not rod:
        return ""

    a = adapter["inertia"]
    adapter_inertia = (
        f'ixx="{a["ixx"]}" ixy="{a["ixy"]}" ixz="{a["ixz"]}" '
        f'iyy="{a["iyy"]}" iyz="{a["iyz"]}" izz="{a["izz"]}"'
    )
    r = rod["inertia"]
    rod_inertia = (
        f'ixx="{r["ixx"]}" ixy="{r["ixy"]}" ixz="{r["ixz"]}" '
        f'iyy="{r["iyy"]}" iyz="{r["iyz"]}" izz="{r["izz"]}"'
    )
    ax, ay, az = adapter["inertial_xyz"]
    ar, ap, ayaw = adapter["inertial_rpy"]
    rx, ry, rz = rod["inertial_xyz"]
    rr, rp, ryaw = rod["inertial_rpy"]
    tx, ty, tz = rod["tcp_xyz"]
    tr, tp, tyaw = rod["tcp_rpy"]

    return f"""
    <!-- Spray gun: adapter {adapter_variant} + rod {rod_variant} -->
    <link name="tool_base_link">
        <inertial>
            <origin xyz="-1.759E-07 0.02497 0.0428" rpy="0 0 0"/>
            <mass value="0.180721738279306"/>
            <inertia ixx="0.000138130675577891" ixy="-2.6261056277312E-10" ixz="-6.35486422323285E-12" iyy="0.000156940897134088" iyz="1.37580712120744E-10" izz="7.74611978883539E-05"/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="package://plasma_urdf_v2/meshes/base/tool_base_link.STL"/></geometry>
            <material name="tool_base_material"><color rgba="0.84706 0.97255 0.78039 1"/></material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="package://plasma_urdf_v2/meshes/base/tool_base_link.STL"/></geometry>
        </collision>
    </link>
    <joint name="link6_to_tool_base" type="fixed">
        <origin xyz="0 0 0" rpy="1.5708 0 1.0472"/>
        <parent link="Link6"/><child link="tool_base_link"/>
    </joint>

    <link name="adapter_link">
        <inertial>
            <origin xyz="{ax} {ay} {az}" rpy="{ar} {ap} {ayaw}"/>
            <mass value="{adapter['mass']}"/>
            <inertia {adapter_inertia}/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="{adapter['mesh']}"/></geometry>
            <material name="adapter_material"><color rgba="1 1 1 1"/></material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="{adapter['mesh']}"/></geometry>
        </collision>
    </link>
    <joint name="tool_base_to_adapter" type="fixed">
        <origin xyz="0 0.078 0" rpy="0 0 0"/>
        <parent link="tool_base_link"/><child link="adapter_link"/>
    </joint>

    <link name="spray_rod_link">
        <inertial>
            <origin xyz="{rx} {ry} {rz}" rpy="{rr} {rp} {ryaw}"/>
            <mass value="{rod['mass']}"/>
            <inertia {rod_inertia}/>
        </inertial>
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="{rod['mesh']}"/></geometry>
            <material name="rod_material"><color rgba="1 1 1 1"/></material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="{rod['mesh']}"/></geometry>
        </collision>
    </link>
    <joint name="adapter_to_spray_rod" type="fixed">
        <origin xyz="0 0.03 0" rpy="0 0 0"/>
        <parent link="adapter_link"/><child link="spray_rod_link"/>
    </joint>

    <link name="tcp_link"/>
    <joint name="spray_rod_to_tcp" type="fixed">
        <origin xyz="{tx} {ty} {tz}" rpy="{tr} {tp} {tyaw}"/>
        <parent link="spray_rod_link"/><child link="tcp_link"/>
    </joint>
"""


def build_printed_part_urdf():
    """Generate URDF XML for the 3D printed calibration tip (260612end.STL, TCP @ 10.8cm)."""
    return """
    <link name="printed_part_link">
        <visual>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="package://plasma_urdf_v2/meshes/end_effector/260612end.STL"/></geometry>
            <material name="printed_material"><color rgba="0.8 0.4 0.2 0.8"/></material>
        </visual>
        <collision>
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <geometry><mesh filename="package://plasma_urdf_v2/meshes/end_effector/260612end.STL"/></geometry>
        </collision>
    </link>
    <joint name="link6_to_printed_part" type="fixed">
        <origin xyz="0 0 0" rpy="0 0 -1.5708"/>
        <parent link="Link6"/><child link="printed_part_link"/>
    </joint>
    <link name="tcp_link"/>
    <joint name="printed_part_to_tcp" type="fixed">
        <origin xyz="0 0 0.108" rpy="0 0 0"/>
        <parent link="printed_part_link"/><child link="tcp_link"/>
    </joint>
"""


def build_srdf_tool_disable_collisions(tool_enabled, use_printed_part):
    """Generate additional SRDF disable_collisions for the selected tool."""
    lines = []
    if tool_enabled:
        lines.extend([
            '<disable_collisions link1="Link6" link2="tool_base_link" reason="Adjacent"/>',
            '<disable_collisions link1="tool_base_link" link2="adapter_link" reason="Adjacent"/>',
            '<disable_collisions link1="adapter_link" link2="spray_rod_link" reason="Adjacent"/>',
            '<disable_collisions link1="spray_rod_link" link2="tcp_link" reason="Adjacent"/>',
        ])
    if use_printed_part:
        lines.extend([
            '<disable_collisions link1="Link6" link2="printed_part_link" reason="Adjacent"/>',
            '<disable_collisions link1="printed_part_link" link2="tcp_link" reason="Adjacent"/>',
            '<disable_collisions link1="printed_part_link" link2="Link5" reason="Never"/>',
        ])
    return "\n    ".join(lines)


def launch_setup(context, *args, **kwargs):
    # Read launch arguments
    tool_enabled = LaunchConfiguration("tool_enabled").perform(context) == "true"
    use_printed_part = LaunchConfiguration("use_printed_part").perform(context) == "true"
    adapter_variant = LaunchConfiguration("adapter_variant").perform(context)
    rod_variant = LaunchConfiguration("rod_variant").perform(context)

    # Build base robot description from xacro
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("rm_eco65_config"),
            "config",
            "rm_eco65.urdf.xacro",
        )
    )

    # Build and inject end effector URDF
    tool_urdf = ""
    if tool_enabled:
        tool_urdf = build_tool_urdf(adapter_variant, rod_variant)
    elif use_printed_part:
        tool_urdf = build_printed_part_urdf()

    robot_xml = robot_description_config.toxml()
    if tool_urdf:
        robot_xml = robot_xml.replace("</robot>", tool_urdf + "\n</robot>")

    robot_description = {"robot_description": robot_xml}

    # Load SRDF and append tool collision pairs
    robot_description_semantic_config = load_file(
        "rm_eco65_config", "config/rm_eco65_description.srdf"
    )
    extra_collisions = build_srdf_tool_disable_collisions(tool_enabled, use_printed_part)
    if extra_collisions and robot_description_semantic_config:
        robot_description_semantic_config = robot_description_semantic_config.replace(
            "</robot>", extra_collisions + "\n</robot>"
        )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Load other configs
    kinematics_yaml = load_yaml("rm_eco65_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Planning pipeline
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "moveit_resources_panda_moveit_config", "config/ompl_planning.yaml"
    )
    if ompl_planning_yaml:
        ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # Trajectory Execution
    moveit_simple_controllers_yaml = load_yaml(
        "rm_eco65_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.15,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # MoveGroup node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory("rm_eco65_config"), "rviz")
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    return [
        rviz_node,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tool_enabled", default_value="false",
                              description="Load plasma spray gun end effector"),
        DeclareLaunchArgument("use_printed_part", default_value="false",
                              description="Load 3D printed calibration tip"),
        DeclareLaunchArgument("adapter_variant", default_value="3_4",
                              description="Adapter variant: 3_4 or 6_8"),
        DeclareLaunchArgument("rod_variant", default_value="3_4_200_1x10",
                              description="Rod variant, e.g. 3_4_150_1x5, 6_8_200_1x10"),
        OpaqueFunction(function=launch_setup),
    ])
