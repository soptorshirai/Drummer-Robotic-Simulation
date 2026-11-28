"""Bring up both descriptions, the trajectory node, and RViz."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def scaled_g1_description(path, scale):
    """Scale the G1's geometry and joint offsets for RViz and IK."""
    if not math.isfinite(scale) or scale <= 0:
        raise ValueError('g1_scale must be a finite positive number')
    if scale == 1.0:
        return path.read_text(encoding='utf-8')

    robot = ET.parse(path).getroot()
    for query in ('joint/origin', 'link/visual/origin', 'link/collision/origin'):
        for origin in robot.findall(query):
            if 'xyz' in origin.attrib:
                origin.set('xyz', scale_dimensions(origin.attrib['xyz'], scale))

    for mesh in robot.findall('.//geometry/mesh'):
        mesh.set('scale', scale_dimensions(mesh.get('scale', '1 1 1'), scale))
    for tag, attributes in (
        ('box', ('size',)), ('sphere', ('radius',)), ('cylinder', ('radius', 'length')),
    ):
        for geometry in robot.findall(f'.//geometry/{tag}'):
            for attribute in attributes:
                geometry.set(attribute, scale_dimensions(geometry.attrib[attribute], scale))

    return ET.tostring(robot, encoding='unicode')


def scale_dimensions(dimensions, scale):
    return ' '.join(format(float(value) * scale, '.12g') for value in dimensions.split())


def kit_robot_description(path):
    robot = ET.parse(path).getroot()
    for mesh in robot.findall('.//mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith('file://./'):
            mesh.set('filename', (path.parent / filename[9:]).resolve().as_uri())
    return ET.tostring(robot, encoding='unicode')


def simulation_nodes(context):
    g1_package = Path(get_package_share_directory('g1_description'))
    g1_urdf = g1_package / 'urdf' / 'g1_29dof.urdf'
    kit_urdf = Path(LaunchConfiguration('kit_urdf').perform(context))
    g1_scale = float(LaunchConfiguration('g1_scale').perform(context))
    g1_description = ParameterValue(scaled_g1_description(g1_urdf, g1_scale), value_type=str)
    kit_description = ParameterValue(kit_robot_description(kit_urdf), value_type=str)

    return [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': g1_description}],
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace='drum_kit', parameters=[{'robot_description': kit_description}],
        ),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            arguments=[
                '--z', LaunchConfiguration('kit_z'),
                '--frame-id', 'pelvis', '--child-frame-id', 'drum_kit_base',
            ],
        ),
        Node(
            package='unitree_drummer', executable='drummer', output='screen',
            parameters=[{
                'robot_description': g1_description,
                'chart': ParameterValue(LaunchConfiguration('chart'), value_type=str),
            }],
        ),
        Node(
            package='rviz2', executable='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ]


def generate_launch_description():
    package = Path(get_package_share_directory('unitree_drummer'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'kit_urdf', default_value=str(Path.cwd() / 'drum_kit' / 'drum_kit.urdf'),
        ),
        DeclareLaunchArgument(
            'g1_scale', default_value='1.10', description='Uniform scale of the simulated G1',
        ),
        DeclareLaunchArgument('kit_z', default_value='-0.65'),
        DeclareLaunchArgument('chart', default_value=str(package / 'config' / 'test_groove.json')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'rviz_config', default_value=str(Path.cwd() / 'rviz' / 'viewurdfplus.rviz'),
        ),
        OpaqueFunction(function=simulation_nodes),
    ])
