#!/usr/bin/env python3
# Copyright 2026, kibo. Apache-2.0
#
# scan_republisher: re-frame the bridged laser scan to 'base_scan'.
#
# Why: when gz-sim converts the TurtleBot3 URDF to SDF, the fixed joints
# (base_footprint -> base_link -> base_scan) are flattened, so the gz lidar
# message is tagged with the sensor's parent link instead of 'base_scan'
# (e.g. "turtlebot3_burger::base_footprint::scan"). The sensor geometry is
# correct - its origin sits where robot_state_publisher places base_scan -
# so re-tagging the frame_id aligns the scan with the ROS TF tree that
# slam_toolbox uses (laser_frame: base_scan).
#
# Topics: subscribes /scan_raw (bridge output), publishes /scan.

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

FRAME_ID = 'base_scan'


class ScanRepublisher(Node):

    def __init__(self):
        super().__init__('scan_republisher')
        self._pub = self.create_publisher(LaserScan, '/scan', 10)
        self._sub = self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, 10)

    def _on_scan(self, msg: LaserScan) -> None:
        msg.header.frame_id = FRAME_ID
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()