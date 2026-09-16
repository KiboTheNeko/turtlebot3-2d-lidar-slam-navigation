#!/usr/bin/env python3
# Copyright 2026, kibo. Apache-2.0
#
# drive_route: closed-loop waypoint driver for headless mapping runs.
#
# Why closed-loop: gz-sim's DiffDrive is open-loop - spin-in-place legs lose
# heading (wheel slip) and straight legs drift, which scatters SLAM wall
# traces. Here each leg is (target_yaw_deg, distance_m); the node
#   a) rotates in place until odometry yaw matches target (P-controller),
#   b) drives straight at vx with a small wz = Kp * yaw_error correction,
#   c) stops after integrating wheel-odometry distance.
# Odometry yaw is drift-prone, but for traversing map coverage it is
# accurate enough - slam_toolbox does the real correction at scan level.
#
# The route below walks every wall of office_arena.sdf inside the lidar
# range (walls at x=+/-2, y=+/-4; LDS max 3.5 m), while dodging the
# furniture (desk_west/desk_east block y +/-1.75..2.65 at x +/-0.35..1.95,
# table_center blocks x +/-0.5 at y 0.6..1.2, cabinet_south blocks
# x +/-0.6 at y -3.45..-2.95, shelf_west blocks x -1.88..-1.48 at
# y -0.7..1.9). The ONLY north-south passage is the west corridor at
# x ~ -1.3 (east of the shelf, west of the table); the route drives it
# twice (out and back):
#   S strip -> SW corner -> SE corner -> E strip ->
#   W corridor north -> NW/NE corners -> return via W corridor.
#
# Usage: ros2 run tb3_office_sim drive_route --ros-args -p use_sim_time:=true

import math
import time as _time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# (target_yaw_deg, distance_m, label) - yaw 0 = +x, +90 = +y, -90 = -y
# Furniture-aware route - every straight leg is cleared against
# office_arena.sdf collision boxes (see comment above).
LEGS = [
    (-90.0, 0.0,   'align  south (heading -y)'),
    (-90.0, 2.85,  'drive south  to (0, -2.85)    [between desk_west and cabinet]'),
    (-180.0, 0.0,  'align  west (heading -x)'),
    (-180.0, 1.3,  'drive west   to (-1.3, -2.85) [south of desk, west of cabinet]'),
    (-90.0, 0.0,   'align  south'),
    (-90.0, 0.95,  'drive south  to (-1.3, -3.80) SW wall corner'),
    (0.0, 0.0,     'align  east (heading +x)'),
    (0.0, 2.6,     'drive east   to (1.3, -3.80)   SE wall corner'),
    (90.0, 0.0,    'align  north (heading +y)'),
    (90.0, 2.80,   'drive north  to (1.3, -1.0)    E strip'),
    (180.0, 0.0,   'align  west'),
    (180.0, 2.6,   'drive west   to (-1.3, -1.0)   W corridor entry'),
    (90.0, 0.0,    'align  north'),
    (90.0, 4.50,   'drive north  to (-1.3, 3.5)    W corridor / NW'),
    (0.0, 0.0,     'align  east'),
    (0.0, 2.6,     'drive east   to (1.3, 3.5)     NE wall corner'),
    (-90.0, 0.0,   'align  south'),
    (-90.0, 0.70,  'drive south  to (1.3, 2.8)'),
    (180.0, 0.0,   'align  west'),
    (180.0, 2.6,   'drive west   to (-1.3, 2.8)'),
    (-90.0, 0.0,   'align  south'),
    (-90.0, 3.85,  'drive south  to (-1.3, -1.05) return via W corridor'),
    (0.0, 0.0,     'align  east'),
    (0.0, 1.3,     'drive east   to (0, -1.05)     mid crossing'),
    (-90.0, 0.0,   'align  south'),
    (-90.0, 1.05,  'drive south  home (0, 0)'),
]

VX = 0.2          # m/s
WZ_MAX = 1.0      # rad/s
YAW_TOL = 0.025   # rad
KP_YAW = 1.5      # 1/s - small enough to not fight wheel dynamics
DIST_TOL = 0.05   # m
RATE_HZ = 30.0


def norm_angle(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


class DriveRoute(Node):

    def __init__(self):
        super().__init__('drive_route')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._odom = None
        self._sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

    def _on_odom(self, msg):
        self._odom = msg

    def _yaw(self):
        o = self._odom.pose.pose.orientation
        return math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                          1.0 - 2.0 * (o.y * o.y + o.z * o.z))

    def _pos(self):
        return (self._odom.pose.pose.position.x,
                self._odom.pose.pose.position.y)

    def _stop(self):
        t = Twist()
        for _ in range(5):
            self._pub.publish(t)

    def _wait_for_odom(self):
        deadline = _time.monotonic() + 30
        while self._odom is None and _time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self._odom is not None

    def _spin_to(self, target_deg, leg_label):
        """Rotate in place until yaw matches target; returns True if done."""
        target = math.radians(target_deg)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._odom is None:
                continue
            err = norm_angle(target - self._yaw())
            if abs(err) < YAW_TOL:
                self._stop()
                return True
            t = Twist()
            t.angular.z = max(-WZ_MAX, min(WZ_MAX, KP_YAW * err))
            self._pub.publish(t)

    def _drive(self, target_deg, distance, leg_label):
        """Drive vx with closed-loop yaw; integrates odometry distance."""
        target = math.radians(target_deg)
        x0, y0 = self._pos()
        travelled = 0.0
        last = (x0, y0)
        while rclpy.ok() and travelled < distance - DIST_TOL:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self._odom is None:
                continue
            cur = self._pos()
            travelled += math.hypot(cur[0] - last[0], cur[1] - last[1])
            last = cur
            err = norm_angle(target - self._yaw())
            t = Twist()
            t.linear.x = VX
            t.angular.z = max(-WZ_MAX, min(WZ_MAX, KP_YAW * err))
            self._pub.publish(t)
        self._stop()
        self.get_logger().info(
            f'leg done: {leg_label} (travelled {travelled:.2f}/{distance} m)')

    def run(self):
        if not self._wait_for_odom():
            self.get_logger().error('no /odom within 30 s - aborting route')
            self._stop()
            return
        rclpy.spin_once(self, timeout_sec=1.0)  # let the first pose settle
        for i, (yaw, dist, label) in enumerate(LEGS):
            self.get_logger().info(
                f'leg {i:02d}/{len(LEGS):02d}: {label} '
                f'(yaw {yaw:+.0f} deg, {dist} m)')
            self._spin_to(yaw, label)
            if dist > 0.0:
                self._drive(yaw, dist, label)
        self._stop()
        self.get_logger().info('route complete - robot stopped')


def main(args=None):
    rclpy.init(args=args)
    node = DriveRoute()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()