from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Elevation mapping node; the RGBD camera must already be publishing."""
    return LaunchDescription([
        Node(
            package='terrain_mapping',
            executable='terrain_mapping_node',
            name='terrain_mapping_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
