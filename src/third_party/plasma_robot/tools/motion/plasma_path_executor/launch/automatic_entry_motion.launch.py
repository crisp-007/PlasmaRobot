import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def load_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return stream.read()


def launch_setup(context):
    tool_share = get_package_share_directory("plasma_tool_description")
    moveit_share = get_package_share_directory("rm_eco65_config")
    robot_xml = xacro.process_file(
        os.path.join(tool_share, "urdf", "rm_eco65_with_plasma_tool.urdf.xacro")
    ).toxml()
    robot_description = {"robot_description": robot_xml}
    robot_description_semantic = {
        "robot_description_semantic": load_text(
            "plasma_tool_description", "config/rm_eco65_plasma.srdf"
        )
    }
    robot_description_kinematics = {
        "robot_description_kinematics": load_yaml(
            "rm_eco65_config", "config/kinematics.yaml"
        )
    }

    ompl = load_yaml("rm_eco65_config", "config/ompl_planning.yaml")
    ompl.update({
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": (
            "default_planner_request_adapters/AddTimeOptimalParameterization "
            "default_planner_request_adapters/FixWorkspaceBounds "
            "default_planner_request_adapters/FixStartStateBounds "
            "default_planner_request_adapters/FixStartStateCollision "
            "default_planner_request_adapters/FixStartStatePathConstraints"
        ),
        "start_state_max_bounds_error": 0.1,
    })
    planning_pipeline = {"move_group": ompl}

    controller_config = load_yaml(
        "rm_eco65_config", "config/moveit_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": controller_config,
        "moveit_controller_manager": (
            "moveit_simple_controller_manager/MoveItSimpleControllerManager"
        ),
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 300.0,
        "trajectory_execution.allowed_goal_duration_margin": 10.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    scene_monitor = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }
    common_parameters = [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        planning_pipeline,
    ]

    control_node = Node(
        package="rm_control",
        executable="rm_control",
        output="screen",
        parameters=[{
            "arm_type": 651,
            "blocking_movej_mode": True,
            "follow": ParameterValue(
                LaunchConfiguration("control_follow"), value_type=bool),
            "point_reached_tolerance_rad": ParameterValue(
                LaunchConfiguration("point_reached_tolerance_rad"), value_type=float),
            "max_command_lead_rad": ParameterValue(
                LaunchConfiguration("max_command_lead_rad"), value_type=float),
            "final_settle_max_error_rad": ParameterValue(
                LaunchConfiguration("final_settle_max_error_rad"), value_type=float),
            "final_settle_speed_percent": ParameterValue(
                LaunchConfiguration("final_settle_speed_percent"), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration("start_rm_control")),
    )
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=common_parameters + [
            trajectory_execution,
            moveit_controllers,
            scene_monitor,
        ],
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="plasma_entry_rviz",
        output="log",
        arguments=["-d", os.path.join(moveit_share, "rviz", "moveit.rviz")],
        parameters=common_parameters,
        condition=IfCondition(LaunchConfiguration("rviz")),
    )
    planner = Node(
        package="plasma_path_executor",
        executable="entry_motion_planner_node",
        name="plasma_entry_motion_planner",
        output="screen",
        parameters=common_parameters + [{
            "motion_enabled": ParameterValue(
                LaunchConfiguration("motion_enabled"), value_type=bool),
            "allow_unvalidated_dry_run": ParameterValue(
                LaunchConfiguration("allow_unvalidated_dry_run"), value_type=bool),
            "velocity_scaling": ParameterValue(
                LaunchConfiguration("velocity_scaling"), value_type=float),
            "acceleration_scaling": ParameterValue(
                LaunchConfiguration("acceleration_scaling"), value_type=float),
            "approach_velocity_scaling": ParameterValue(
                LaunchConfiguration("approach_velocity_scaling"), value_type=float),
            "approach_acceleration_scaling": ParameterValue(
                LaunchConfiguration("approach_acceleration_scaling"), value_type=float),
            "complete_plan_attempts": ParameterValue(
                LaunchConfiguration("complete_plan_attempts"), value_type=int),
            "max_joint_step_rad": ParameterValue(
                LaunchConfiguration("max_joint_step_rad"), value_type=float),
            "required_collision_stage": ParameterValue(
                LaunchConfiguration("required_collision_stage"), value_type=int),
            "max_plan_age_sec": ParameterValue(
                LaunchConfiguration("max_plan_age_sec"), value_type=float),
            "joint_state_wait_sec": ParameterValue(
                LaunchConfiguration("joint_state_wait_sec"), value_type=float),
        }],
    )
    return [control_node, robot_state_publisher, move_group, rviz, planner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("motion_enabled", default_value="false"),
        DeclareLaunchArgument("allow_unvalidated_dry_run", default_value="false"),
        DeclareLaunchArgument("velocity_scaling", default_value="0.10"),
        DeclareLaunchArgument("acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("approach_velocity_scaling", default_value="0.10"),
        DeclareLaunchArgument("approach_acceleration_scaling", default_value="0.10"),
        DeclareLaunchArgument("complete_plan_attempts", default_value="8"),
        DeclareLaunchArgument("max_joint_step_rad", default_value="0.17453292519943295"),
        DeclareLaunchArgument("required_collision_stage", default_value="8"),
        DeclareLaunchArgument("max_plan_age_sec", default_value="600.0"),
        DeclareLaunchArgument("joint_state_wait_sec", default_value="15.0"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_rm_control", default_value="true"),
        DeclareLaunchArgument("control_follow", default_value="false"),
        DeclareLaunchArgument(
            "point_reached_tolerance_rad", default_value="0.008726646259971648"),
        DeclareLaunchArgument(
            "max_command_lead_rad", default_value="0.03490658503988659"),
        DeclareLaunchArgument(
            "final_settle_max_error_rad", default_value="0.03490658503988659"),
        DeclareLaunchArgument("final_settle_speed_percent", default_value="10"),
        OpaqueFunction(function=launch_setup),
    ])
