#!/usr/bin/env python3
"""Republish the controller odometry (~/odom) as the odom -> trunk TF."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.declare_parameter('odom_topic', '/mevius_mpc_controller/odom')
        odom_topic = self.get_parameter('odom_topic').value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 50)

    def odom_cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id      # "odom"
        t.child_frame_id = msg.child_frame_id         # "trunk"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
