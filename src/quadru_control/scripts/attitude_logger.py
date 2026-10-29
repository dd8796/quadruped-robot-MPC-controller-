#!/usr/bin/env python3
"""Log the trunk pose from the controller odometry to CSV (t, x, y, z, roll, pitch, yaw in deg).

Usage: ros2 run quadru_control attitude_logger.py --ros-args -p output:=/path/file.csv
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class AttitudeLogger(Node):
    def __init__(self):
        super().__init__('attitude_logger')
        self.declare_parameter('output', 'attitude.csv')
        self.declare_parameter('period', 0.02)
        self.period = self.get_parameter('period').value
        self.out = open(self.get_parameter('output').value, 'w', buffering=1)
        self.out.write('t,x,y,z,roll,pitch,yaw\n')
        self.last = -1.0
        self.create_subscription(Odometry, '/mevius_mpc_controller/odom', self.cb, 50)

    def cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t - self.last < self.period:
            return
        self.last = t
        q = msg.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        self.out.write(f'{t:.3f},{p.x:.4f},{p.y:.4f},{p.z:.4f},'
                       f'{math.degrees(roll):.2f},{math.degrees(pitch):.2f},{math.degrees(yaw):.2f}\n')


def main():
    rclpy.init()
    rclpy.spin(AttitudeLogger())


if __name__ == '__main__':
    main()
