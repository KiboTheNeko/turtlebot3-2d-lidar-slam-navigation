#!/usr/bin/env python3
# Copyright 2026, kibo. Apache-2.0
#
# odom_to_tf: republish the bridged /odom (nav_msgs/Odometry) as a TF
# transform odom -> base_footprint.
#
# Why this node exists: Gazebo Harmonic publishes wheel odometry on the gz
# transport bus; ros_gz_bridge converts it to nav_msgs/Odometry but has no
# channel for transforms. slam_toolbox (and RViz) need the odom -> base_footprint
# edge of the TF tree, so we broadcast it from the odometry message.
# This is the full "robot TF bridge" analog of gazebo_ros's odometry TF
# publishing in classic Gazebo: a ~15-line node replaces an EKF.

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """Broadcast odom->base_footprint from the /odom message."""

    def __init__(self):
        super().__init__('odom_to_tf')
        self._tf_broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        t = TransformStamped()
        # Stamp with the odometry SAMPLING time (msg.header.stamp), not
        # get_clock().now(): the transform is a snapshot at that instant.
        # Stamping with a later "now" creates a forward-extrapolated TF,
        # which (when /clock jumps, as gz-sim does at world load) makes the
        # TF buffer see "jump back in time", clear itself, and slam_toolbox
        # then SIGABRTs on the resulting extrapolation lookups.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()