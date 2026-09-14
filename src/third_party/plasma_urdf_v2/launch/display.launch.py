import os
import subprocess
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    package_name = 'plasma_urdf_v2'
    pkg_share = get_package_share_directory(package_name)

    adapter_type = LaunchConfiguration('adapter_type').perform(context)
    spray_rod_type = LaunchConfiguration('spray_rod_type').perform(context)

    yaml_file = os.path.join(pkg_share, 'config', 'tool_variants.yaml')
    xacro_file = os.path.join(pkg_share, 'urdf', 'plasma_urdf_v2.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'display.rviz')
    
    print(f"[DEBUG] rviz_file 路径: {rviz_file}")

    cfg = load_yaml(yaml_file)

    if adapter_type not in cfg['adapters']:
        raise RuntimeError(f'未知 adapter_type: {adapter_type}')

    if adapter_type not in cfg['spray_rods']:
        raise RuntimeError(f'spray_rods 中不存在 adapter_type: {adapter_type}')

    if spray_rod_type not in cfg['spray_rods'][adapter_type]:
        raise RuntimeError(
            f'在 adapter_type={adapter_type} 下，不存在 spray_rod_type={spray_rod_type}'
        )

    adapter_cfg = cfg['adapters'][adapter_type]
    rod_cfg = cfg['spray_rods'][adapter_type][spray_rod_type]

    # 打印初始参数和最终选择结果
    print("\n" + "="*60)
    print("[plasma_urdf_v2] 工具规格选择信息")
    print("="*60)
    print(f"==========目标工件==========")
    print(f"adapter_type: {adapter_type}")
    print(f"spray_rod_type: {spray_rod_type}")
    print(f"==========当前工件==========")
    print(f"adapter mesh: {adapter_cfg['mesh']}")
    print(f"spray_rod mesh: {rod_cfg['mesh']}")
    print(f"TCP 位置: {rod_cfg['tcp_xyz']}")
    print(f"TCP 姿态: {rod_cfg['tcp_rpy']}")
    print("="*60 + "\n")

    cmd = ''.join([
        'xacro ', xacro_file, ' ',
        'adapter_mesh:=', adapter_cfg['mesh'], ' ',
        'spray_rod_mesh:=', rod_cfg['mesh'], ' ',

        'adapter_mass:=', str(adapter_cfg['mass']), ' ',
        'adapter_inertial_xyz:="', ' '.join(map(str, adapter_cfg['inertial_xyz'])), '" ',
        'adapter_inertial_rpy:="', ' '.join(map(str, adapter_cfg['inertial_rpy'])), '" ',
        'adapter_ixx:=', str(adapter_cfg['inertia']['ixx']), ' ',
        'adapter_ixy:=', str(adapter_cfg['inertia']['ixy']), ' ',
        'adapter_ixz:=', str(adapter_cfg['inertia']['ixz']), ' ',
        'adapter_iyy:=', str(adapter_cfg['inertia']['iyy']), ' ',
        'adapter_iyz:=', str(adapter_cfg['inertia']['iyz']), ' ',
        'adapter_izz:=', str(adapter_cfg['inertia']['izz']), ' ',

        'spray_rod_mass:=', str(rod_cfg['mass']), ' ',
        'spray_rod_inertial_xyz:="', ' '.join(map(str, rod_cfg['inertial_xyz'])), '" ',
        'spray_rod_inertial_rpy:="', ' '.join(map(str, rod_cfg['inertial_rpy'])), '" ',
        'spray_rod_ixx:=', str(rod_cfg['inertia']['ixx']), ' ',
        'spray_rod_ixy:=', str(rod_cfg['inertia']['ixy']), ' ',
        'spray_rod_ixz:=', str(rod_cfg['inertia']['ixz']), ' ',
        'spray_rod_iyy:=', str(rod_cfg['inertia']['iyy']), ' ',
        'spray_rod_iyz:=', str(rod_cfg['inertia']['iyz']), ' ',
        'spray_rod_izz:=', str(rod_cfg['inertia']['izz']), ' ',

        'tcp_xyz:="', ' '.join(map(str, rod_cfg['tcp_xyz'])), '" ',
        'tcp_rpy:="', ' '.join(map(str, rod_cfg['tcp_rpy'])), '" ',
    ])

    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(f'xacro 执行失败:\n{result.stderr}')
    robot_description_str = result.stdout

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_str}]
    )

    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_file]
    )

    return [
        robot_state_publisher_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'adapter_type',
            default_value='6_8',
            description='转接件类型，如 3_4 或 6_8'
        ),
        DeclareLaunchArgument(
            'spray_rod_type',
            default_value='200_1x10',
            description='喷杆类型，如 150_1x5、150_1x10、200_1x20'
        ),
        OpaqueFunction(function=launch_setup)
    ])