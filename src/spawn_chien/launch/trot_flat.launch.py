"""Fast trot on flat ground: empty world, no terrain perception, config/trot_flat.yaml.

ros2 launch spawn_chien trot_flat.launch.py [speed:=0.8] [gui:=true] [rviz:=false] [dashboard:=false]
"""
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONTROLLER = 'mevius_mpc_controller'


def _spawn(context):
    pkg_share = get_package_share_directory('spawn_chien')
    with open(os.path.join(pkg_share, 'config', 'trot_flat.yaml')) as f:
        config = yaml.safe_load(f)
    speed = LaunchConfiguration('speed').perform(context)
    if speed:
        config[CONTROLLER]['ros__parameters']['walk_speed'] = float(speed)
    fd, path = tempfile.mkstemp(prefix='trot_flat_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(config, f)
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'spawn.launch.py')),
        launch_arguments={
            'controllers': path,
            'terrain_aware': 'false',
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'speed', default_value='',
            description='Walking speed (m/s); empty: walk_speed of config/trot_flat.yaml.'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument(
            'dashboard', default_value='false',
            description='Open the real-time control dashboard (quadru_control).'),
        OpaqueFunction(function=_spawn),
        Node(package='quadru_control', executable='control_dashboard.py', output='screen',
             condition=IfCondition(LaunchConfiguration('dashboard'))),
    ])
