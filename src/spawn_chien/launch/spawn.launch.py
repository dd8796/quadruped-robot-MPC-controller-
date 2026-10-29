import os
import re

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('spawn_chien')
    urdf_path = os.path.join(pkg_share, 'urdf', 'urdf_chien.urdf')
    controllers_yaml_path = os.path.join(pkg_share, 'config', 'mevius_controllers.yaml')

    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    # gazebo_ros2_control passes robot_description through the ROS argument parser, which rejects
    # XML comments containing "word: ". Publish a comment-free description.
    robot_description_for_control = re.sub(
        r'<!--.*?-->', '', robot_description, flags=re.DOTALL)

    # Single locomotion controller (config/mevius_controllers.yaml).
    controller = 'mevius_mpc_controller'

    # Spawn height puts the 20 mm foot spheres on the ground at the home pose.
    declare_args = [
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='-0.20727654'),
        DeclareLaunchArgument('roll', default_value='0.0'),
        DeclareLaunchArgument('pitch', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'world', default_value=[FindPackageShare('gazebo_ros'), '/worlds/empty.world'],
            description='Gazebo world file.'),
        DeclareLaunchArgument(
            'terrain_aware', default_value='false',
            description='Start terrain mapping and enable terrain-aware locomotion.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Open RViz2 (camera cloud, image and elevation map).'),
        DeclareLaunchArgument(
            'controllers', default_value=controllers_yaml_path,
            description='controller_manager / locomotion parameters file.'),
    ]

    # Gazebo resolves model://spawn_chien/... from GAZEBO_MODEL_PATH: prepend this package's parent.
    gazebo_model_path = os.pathsep.join(filter(None, [
        os.path.dirname(pkg_share),
        os.environ.get('GAZEBO_MODEL_PATH'),
    ]))
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH', value=gazebo_model_path)
    gazebo_plugin_path = os.pathsep.join(filter(None, [
        os.path.join(get_package_prefix('spawn_chien'), 'lib'),
        os.environ.get('GAZEBO_PLUGIN_PATH'),
    ]))
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', value=gazebo_plugin_path)

    gazebo_launch = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzserver.launch.py')),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_launch, 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    # TF from /joint_states (joint_state_broadcaster).
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_for_control,
            'use_sim_time': True,
        }],
    )

    # Spawns the robot, with CONTROLLERS_YAML_PATH replaced by the real config path.
    spawn_node = Node(
        package='spawn_chien',
        executable='spawn_robot.py',
        name='spawn_chien',
        output='screen',
        arguments=['--urdf', urdf_path, '--name', 'chien',
                   '--controllers-yaml', LaunchConfiguration('controllers'),
                   '--x', LaunchConfiguration('x'),
                   '--y', LaunchConfiguration('y'),
                   '--z', LaunchConfiguration('z'),
                   '--roll', LaunchConfiguration('roll'),
                   '--pitch', LaunchConfiguration('pitch'),
                   '--yaw', LaunchConfiguration('yaw')],
    )

    # Started once the robot is spawned (controller_manager is then loaded). The spawner talks to
    # controller_manager directly, unlike the ros2cli daemon (rclpy issue 1221).
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '60.0',
            '--service-call-timeout', '60.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )
    load_robot_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            controller,
            '--controller-manager-timeout', '60.0',
            '--service-call-timeout', '60.0',
            '--switch-timeout', '30.0',
        ],
        output='screen',
    )

    # odom -> trunk TF from the controller odometry.
    odom_to_tf_node = Node(
        package='spawn_chien',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'odom_topic': f'/{controller}/odom'}],
    )

    # terrain_aware:=true starts terrain mapping and enables the controller parameter.
    terrain_mapping_node_action = Node(
        package='terrain_mapping',
        executable='terrain_mapping_node',
        name='terrain_mapping_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('terrain_aware')),
    )
    set_terrain_aware_param = ExecuteProcess(
        cmd=['ros2', 'param', 'set', f'/{controller}', 'terrain_aware',
             LaunchConfiguration('terrain_aware')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('terrain_aware')),
    )

    rviz_config_path = os.path.join(pkg_share, 'config', 'terrain_camera.rviz')
    # Clear the VS Code snap GTK/GIO variables for rviz2 only: they load an incompatible libpthread.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        additional_env={
            'GTK_PATH': '',
            'GTK_EXE_PREFIX': '',
            'GTK_IM_MODULE_FILE': '',
            'GDK_PIXBUF_MODULE_FILE': '',
            'GDK_PIXBUF_MODULEDIR': '',
            'GIO_MODULE_DIR': '',
            'GSETTINGS_SCHEMA_DIR': '',
            'LOCPATH': '',
        },
    )

    return LaunchDescription(declare_args + [
        set_gazebo_model_path,
        set_gazebo_plugin_path,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_robot_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_robot_controller,
                on_exit=[set_terrain_aware_param],
            )
        ),
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        spawn_node,
        odom_to_tf_node,
        terrain_mapping_node_action,
        rviz_node,
    ])
