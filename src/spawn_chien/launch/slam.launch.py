"""LiDAR 3D -> scan 2D -> SLAM (slam_toolbox). A lancer APRES spawn.launch.py (le robot doit
deja exister dans Gazebo et etre en train de marcher -- ce fichier ne spawn rien lui-meme).

pointcloud_to_laserscan n'est pas installable ici (pas de sudo) : .deb extrait a la main dans
quadru_ws/vendor/ (cf. vendor/setup_vendor.bash, a sourcer AVANT ce launch). Sur une machine avec
sudo, `apt install ros-humble-pointcloud-to-laserscan` suffit et vendor/ n'est pas necessaire.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('spawn_chien')
    slam_params = os.path.join(pkg_share, 'config', 'slam_params.yaml')

    # 3D cloud -> 2D scan: keep the near-horizontal band (obstacles), drop ground and ceiling.
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        remappings=[('cloud_in', '/chien/points'), ('scan', '/scan')],
        parameters=[{
            'target_frame': '',
            'transform_tolerance': 0.02,
            'min_height': -0.15,
            'max_height': 0.15,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.006981,   # 2*pi/900 (900 horizontal samples)
            'scan_time': 0.1,              # 10 Hz lidar
            'range_min': 0.15,
            'range_max': 25.0,
            'use_inf': True,
            'concurrency_level': 1,
        }],
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    return LaunchDescription([
        pointcloud_to_scan_node,
        slam_toolbox_node,
    ])
