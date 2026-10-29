#!/usr/bin/env python3
"""Simple ROS2 node to call gazebo's SpawnEntity service with a URDF file."""
import math
import os
import re
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import argparse
import sys


def quaternion_from_euler(roll, pitch, yaw):
    """Roll/pitch/yaw (rad) -> quaternion (x, y, z, w), as tf_transformations."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Spawner(Node):
    def __init__(self, name, urdf_path, service_name, controllers_yaml=None,
                 x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        super().__init__('spawn_robot')
        self.cli = self.create_client(SpawnEntity, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        if not urdf_path or not os.path.exists(urdf_path):
            self.get_logger().error(f'URDF file not found: {urdf_path}')
            raise RuntimeError('URDF not found')

        with open(urdf_path, 'r') as f:
            xml = f.read()

        # gazebo_ros2_control passes the URDF through rcl's argument parser, which fails on XML
        # comments containing "word: ". Strip comments before spawning.
        xml = re.sub(r'<!--.*?-->', '', xml, flags=re.DOTALL)

        # The plugin needs a real path for its parameters file (package:// is not resolved).
        if 'CONTROLLERS_YAML_PATH' in xml:
            if not controllers_yaml or not os.path.exists(controllers_yaml):
                self.get_logger().error(
                    f'controllers YAML not found: {controllers_yaml}')
                raise RuntimeError('controllers YAML not found')
            xml = xml.replace('CONTROLLERS_YAML_PATH', controllers_yaml)

        req = SpawnEntity.Request()
        req.name = name
        req.xml = xml
        req.robot_namespace = ''
        req.initial_pose = Pose()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw
        req.reference_frame = 'world'
        self.get_logger().info(
            f'Spawn pose: xyz=({x}, {y}, {z}) rpy=({roll}, {pitch}, {yaw})')

        self.get_logger().info(f'Calling spawn service ({service_name}) for {name}...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if result is None or not result.success:
            self.get_logger().error('Failed to spawn entity')
            raise RuntimeError('SpawnEntity failed')
        self.get_logger().info(f'Spawn result: {result}')

        # HomePosePlugin holds the stance until ros2_control takes over; keep Gazebo running.


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--urdf', help='Path to URDF file', default='urdf_chien.urdf')
    parser.add_argument('--name', help='Name of the spawned entity', default='chien')
    parser.add_argument('--service', help='Spawn service name', default='spawn_entity')
    parser.add_argument('--controllers-yaml',
                         help='Absolute path to the gazebo_ros2_control controllers YAML',
                         default=None)
    # Lowest foot center 227.277 mm below the root at the home pose: z = -0.207277 puts the 20 mm
    # foot sphere on the ground.
    parser.add_argument('--x', type=float, default=0.0)
    parser.add_argument('--y', type=float, default=0.0)
    parser.add_argument('--z', type=float, default=-0.20727654)
    parser.add_argument('--roll', type=float, default=0.0)
    parser.add_argument('--pitch', type=float, default=0.0)
    parser.add_argument('--yaw', type=float, default=0.0)
    args, unknown = parser.parse_known_args(argv)

    rclpy.init(args=unknown)
    try:
        node = Spawner(args.name, args.urdf, args.service, args.controllers_yaml,
                        args.x, args.y, args.z, args.roll, args.pitch, args.yaw)
    except Exception as e:
        print('Error:', e)
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main(sys.argv[1:])
