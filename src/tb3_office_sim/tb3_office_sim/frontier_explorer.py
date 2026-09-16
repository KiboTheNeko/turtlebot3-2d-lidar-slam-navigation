#!/usr/bin/env python3
# Copyright 2026, kibo. Apache-2.0
#
# frontier_explorer: arena-generic autonomous exploration via Nav2.
#
# Why: the v1 pipeline (drive_route.py) is a hand-planned 26-leg route that
# only works in office_arena.sdf. This node replaces that for v2: it works in
# ANY arena because it reads the live occupancy grid instead of a hardcoded
# plan.
#
# Algorithm (greedy frontier exploration):
#   1. Subscribe to /map (transient_local) from slam_toolbox.
#   2. SAFE free cells = FREE (0) cells at least SAFE_CLEAR_M from any occupied
#      cell (wall strip filtered out - Nav2 aborts goals on/near the lethal
#      inflated band of the arena walls).
#   3. Frontier cells = UNKNOWN (-1) cells adjacent (8-neighbour) to a SAFE free
#      cell. Cluster into connected components; centroid of the component's
#      SAFE free neighbours = a definitely drivable point.
#   4. Send NavigateToPose to the FARTHEST safe goal (centroid-preferred).
#      Nearest-first is a deadlock here: at a furniture-cramped spawn the whole
#      frontier is one ring around the robot, its centroid sits on the robot,
#      and Nav2 "reaches" it without moving - so the map never grows.
#   5. If a goal is aborted or times out, mark the region bad (0.35 m recall)
#      and try the next one; forgive all bans after a budget of retries so a
#      geometrically sealed pocket cannot block completion forever.
#   6. Stop when 3 consecutive map updates show no frontiers -> log coverage.
#
# The /scan -> /map pipeline stays live the whole time: slam_toolbox keeps
# building the map while Nav2 drives (costmaps subscribe to the same /map).
#
# NOTE: this node uses `rclpy.spin()` + a timer callback (not spin_once in a
# loop) because rclpy's spin_once silently swallows subscription callbacks on
# nodes with use_sim_time:=True.
#
# Usage:
#   ros2 run tb3_office_sim frontier_explorer --ros-args \
#     -p use_sim_time:=true -p arena_size:="5 5"
#   (arena_size optional: only used for the coverage % estimate)

import math
import time as _time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSDurabilityPolicy,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener

UNKNOWN = -1
FREE = 0
NO_GOAL_CHECKS = 3        # consecutive empty frontier maps before we stop
GOAL_TIMEOUT_S = 60.0     # per-goal wall-clock timeout -> cancel + skip
GOAL_MIN_SPACING_M = 0.5  # don't send two goals closer than this
BAD_RECALL_M = 0.35       # goals within this of a failed goal are skipped too
MIN_OUTWARD_M = 0.6       # centroid must beat this to be preferred over extremes
SAFE_CLEAR_M = 0.30       # goals stay >= this far from occupied (wall strip)
FORGIVE_LIMIT = 6         # full bad-goal forgives before "sealed -> complete"
RATE_HZ = 2.0


def _yaw_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('arena_size', '')
        self._arena_size = self._parse_arena_size()

        # --- map (transient_local: slam_toolbox latches /map) -------------
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos)
        self._map = None

        # --- odom (robot pose in odom frame) ------------------------------
        self._odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)
        self._odom = None

        # --- TF (map -> base_footprint for "nearest frontier" selection) --
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # --- Nav2 action client -------------------------------------------
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        self._result_future = None
        self._goal_pose = None      # (x, y) currently being pursued
        self._goal_sent_time = 0.0
        self._no_goal_checks = 0
        self._goals_sent = 0
        self._bad_goals = set()     # (x,y) rounded -> skip on retry
        self._stale_ticks = 0       # ticks where every frontier was bad-banned
        self._forgives = 0          # forgive-budget for sealed pockets

        # --- state machine (driven by timer, called from rclpy.spin()) ----
        self._phase = 'WAIT_NAV2'
        self._tick_timer = self.create_timer(1.0 / RATE_HZ, self._on_tick)
        self.get_logger().info('waiting for Nav2 navigate_to_pose ...')

    # ------------------------------------------------------------------ map
    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _on_odom(self, msg: Odometry):
        self._odom = msg

    def _parse_arena_size(self):
        s = self.get_parameter('arena_size').value
        if not s:
            return None
        parts = s.split()
        if len(parts) == 2:
            try:
                return (float(parts[0]), float(parts[1]))
            except ValueError:
                pass
        self.get_logger().warn(
            f'arena_size="{s}" not "<W> <H>", coverage estimate disabled')
        return None

    # -------------------------------------------------------------- helpers
    def _robot_in_map(self):
        """(x, y) of base_footprint in the map frame, or None."""
        try:
            t = self._tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    # ---------------------------------------------------------- state machine
    def _on_tick(self):
        """Timer callback: drives the exploration state machine via rclpy.spin()"""
        if self._phase == 'WAIT_NAV2':
            # non-blocking check (wait_for_server with timeout=0 returns immediately)
            if self._nav.wait_for_server(timeout_sec=0.0):
                self._phase = 'WAIT_MAP'
                self.get_logger().info('Nav2 up. Waiting for first map ...')

        elif self._phase == 'WAIT_MAP':
            if self._map is not None:
                self._phase = 'EXPLORE'
                self.get_logger().info('first map received - exploration starting')

        elif self._phase == 'EXPLORE':
            self._tick_explore()

        elif self._phase == 'DONE':
            self._tick_timer.cancel()
            self.get_logger().info('exploration finished, node idle')

    def _frontier_goals(self, occ: OccupancyGrid):
        """Return (safe_goals, all_goals) for the current map.

        safe_goals: one centroid per frontier component (mean of its SAFE free
                   neighbours). These sit in open free space, >= SAFE_CLEAR_M
                   from any occupied cell, and are the preferred targets.
        all_goals:  safe_goals PLUS each component's principal-axis extremes.
                   The extremes are used only to escape a furniture-cramped
                   spawn: there the whole frontier is often ONE ring-shaped
                   component whose centroid lands right on top of the robot (a
                   goal Nav2 reaches without moving -> deadlock). The extremes
                   point outward and guarantee a real first move.

        "SAFE free" = FREE cell with no occupied cell within SAFE_CLEAR_M.
        Filtering the wall-adjacent strip this way is what keeps Nav2 from
        aborting: goals on the lethal inflated band of a wall (or a transient
        out-of-bounds costmap during map growth) make the planner fail with
        "worldToMap failed" / "Failed to create plan".
        """
        w, h = occ.info.width, occ.info.height
        data = np.asarray(occ.data, dtype=np.int8).reshape((h, w))
        unknown = data == UNKNOWN
        free = data == FREE
        occupied = data == 100

        # erode free by SAFE_CLEAR_M around occupied cells
        res = occ.info.resolution
        erode = max(1, int(round(SAFE_CLEAR_M / res)))
        occ_pad = np.pad(occupied, erode, mode='constant', constant_values=True)
        safe_free = np.full((h, w), True, dtype=bool)
        for dy in range(-erode, erode + 1):
            for dx in range(-erode, erode + 1):
                safe_free &= ~occ_pad[erode + dy:erode + dy + h,
                                      erode + dx:erode + dx + w]
        safe_free &= free

        # frontier mask: unknown cell with >=1 safe-free 8-neighbour
        sf_pad = np.pad(safe_free, 1, mode='constant', constant_values=False)
        unknown_pad = np.pad(unknown, 1, mode='constant', constant_values=False)
        has_sf_nb = np.zeros((h, w), dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                has_sf_nb |= sf_pad[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        frontier = unknown_pad[1:1 + h, 1:1 + w] & has_sf_nb

        if not frontier.any():
            return [], []

        ox = occ.info.origin.position.x
        oy = occ.info.origin.position.y

        def _extreme_points(s):
            """Two free cells at the extremes of the set's principal axis."""
            if len(s) < 3:
                return []
            d = s - s.mean(axis=0)
            eig = np.linalg.eigh(d.T @ d)[1][:, -1]   # principal direction
            proj = d @ eig
            return [tuple(s[i]) for i in (int(proj.argmax()), int(proj.argmin()))]

        def _cell_world(c):
            return (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res)

        safe, all_goals = [], []
        visited = np.zeros_like(frontier, dtype=bool)
        for y0 in range(h):
            for x0 in range(w):
                if not frontier[y0, x0] or visited[y0, x0]:
                    continue
                # BFS this component
                comp = []
                queue = [(y0, x0)]
                visited[y0, x0] = True
                while queue:
                    y, x = queue.pop()
                    comp.append((y, x))
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            if dy == 0 and dx == 0:
                                continue
                            ny, nx = y + dy, x + dx
                            if (0 <= ny < h and 0 <= nx < w
                                    and frontier[ny, nx] and not visited[ny, nx]):
                                visited[ny, nx] = True
                                queue.append((ny, nx))
                # all SAFE-free 8-neighbours of the component
                fn = []
                for (y, x) in comp:
                    for dy in (-1, 0, 1):
                        for dx in (-1, 0, 1):
                            ny, nx = y + dy, x + dx
                            if (0 <= ny < h and 0 <= nx < w
                                    and safe_free[ny, nx]):
                                fn.append((nx, ny))
                if not fn:
                    continue
                fn = list(set(fn))  # dedupe cell visitors
                s = np.asarray(fn, dtype=np.float64)
                cxy = (s[:, 0].mean(), s[:, 1].mean())
                cand = [_cell_world(cxy)] + [
                    _cell_world(c) for c in _extreme_points(s)]
                for g in cand:
                    if not self._is_bad(*g):
                        all_goals.append(g)
                if not self._is_bad(*_cell_world(cxy)):
                    safe.append(_cell_world(cxy))
        return safe, all_goals

    def _is_bad(self, x, y):
        """True if a goal is within BAD_RECALL_M of a previously failed one."""
        return any(math.hypot(x - bx, y - by) < BAD_RECALL_M
                   for (bx, by) in self._bad_goals)

    def _pick_goal(self, safe_goals, all_goals):
        """Farthest drivable goal, preferring safe centroids.

        Farthest-first (not nearest-first) is deliberate: at a furniture-cramped
        spawn the whole frontier is often a single ring-shaped component around
        the robot, so "nearest" is a point right on top of the robot and Nav2
        reaches it without moving -> the map never grows and the explorer
        deadlocks re-sending the same goal. Pushing outward to the farthest
        candidate guarantees motion and opens new space.

        Centroids are preferred because they sit in open free space; the
        wall-hugging extremes are only used when the best centroid is too close
        to the robot to produce a real move (the cramped-spawn case).
        """
        robot = self._robot_in_map()
        pool = safe_goals if safe_goals else all_goals
        if not pool:
            return None          # frontier vanished mid-tick; try again
        if robot is None:
            return pool[0]
        dist = [(math.hypot(g[0] - robot[0], g[1] - robot[1]), g)
                for g in pool]
        dist.sort(reverse=True)
        if dist[0][0] >= MIN_OUTWARD_M or not all_goals:
            return dist[0][1]
        # best centroid is still on top of us: use the outward extremes
        ext = [(math.hypot(g[0] - robot[0], g[1] - robot[1]), g)
               for g in all_goals]
        ext.sort(reverse=True)
        return ext[0][1]

    # -------------------------------------------------------------- action
    def _cancel_goal(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._goal_handle = None
        self._result_future = None

    def _send_goal(self, x, y):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation = _yaw_quat(0.0)
        self.get_logger().info(f'-> target frontier ({x:.2f}, {y:.2f})')
        send_future = self._nav.send_goal_async(goal)
        send_future.add_done_callback(self._on_goal_accepted)

    def _on_goal_accepted(self, future):
        ghandle = future.result()
        if ghandle is None:
            self.get_logger().warn('goal rejected by Nav2')
            self._mark_bad_and_clear()
            return
        self._goal_handle = ghandle
        self._goal_sent_time = _time.monotonic()
        self._result_future = ghandle.get_result_async()
        self.get_logger().info('goal accepted, navigating...')

    def _mark_bad_and_clear(self):
        if self._goal_pose is not None:
            self._bad_goals.add((round(self._goal_pose[0], 1),
                                 round(self._goal_pose[1], 1)))
        self._goal_pose = None
        self._goal_handle = None
        self._result_future = None

    def _log_coverage(self):
        if self._map is None:
            return
        w, h = self._map.info.width, self._map.info.height
        data = np.asarray(self._map.data, dtype=np.int8).reshape((h, w))
        res = self._map.info.resolution
        free_n = int(np.sum(data == FREE))
        free_area = free_n * res * res
        self.get_logger().info(
            f'[coverage] free={free_area:.2f} m^2 | goals sent: {self._goals_sent}')
        if self._arena_size:
            aw, ah = self._arena_size
            self.get_logger().info(
                f'[coverage] ~{min(free_area / (aw * ah) * 100.0, 100.0):.1f}% '
                f'of the {aw:.0f}x{ah:.0f} m arena floor touched by the map')

    def _tick_explore(self):
        """Exploration tick: check in-flight goal or pick a new one."""
        # --- a goal is in flight: wait for completion/timeout ------------
        if self._result_future is not None:
            if self._result_future.done():
                status = self._result_future.result().status
                if status != 4:
                    self.get_logger().warn(f'goal ended status {status} '
                                           '(4=SUCCEEDED) - skipping it')
                    self._mark_bad_and_clear()
                else:
                    self.get_logger().info('goal reached')
                    self._log_coverage()
                    self._goal_pose = None
                    self._goal_handle = None
                    self._result_future = None
            elif (_time.monotonic() - self._goal_sent_time) > GOAL_TIMEOUT_S:
                self.get_logger().warn(
                    f'goal timed out after {GOAL_TIMEOUT_S:.0f}s - cancelling')
                self._cancel_goal()
                self._mark_bad_and_clear()
            return

        # --- no goal in flight: is there anywhere left to explore? -------
        safe_goals, all_goals = self._frontier_goals(self._map)
        if not all_goals:
            self._no_goal_checks += 1
            self.get_logger().info(
                f'no frontiers ({self._no_goal_checks}/{NO_GOAL_CHECKS})')
            if self._no_goal_checks >= NO_GOAL_CHECKS:
                self._log_coverage()
                self.get_logger().info(
                    'EXPLORATION COMPLETE - no frontiers remaining')
                self._phase = 'DONE'
            return

        self._no_goal_checks = 0

        # If every remaining frontier candidate is bad-banned (frontier exists
        # but no usable goal), forgive old failures so a pocket that became
        # reachable again can be retried. A bounded budget of forgives keeps a
        # geometrically sealed pocket from blocking completion forever.
        if not safe_goals:
            self._stale_ticks += 1
            if self._stale_ticks >= 2:
                if self._forgives >= FORGIVE_LIMIT:
                    self.get_logger().warn(
                        'frontier exists but goals keep failing after '
                        f'{FORGIVE_LIMIT} forgives - sealed pocket excluded')
                    self._log_coverage()
                    self.get_logger().info(
                        'EXPLORATION COMPLETE - no reachable frontiers remain')
                    self._phase = 'DONE'
                    return
                self._forgives += 1
                self.get_logger().warn(
                    f'all {len(self._bad_goals)} bad goals forgiven '
                    f'(#{self._forgives}) - retrying')
                self._bad_goals.clear()
                self._stale_ticks = 0
                safe_goals, all_goals = self._frontier_goals(self._map)
        else:
            self._stale_ticks = 0

        target = self._pick_goal(safe_goals, all_goals)
        if target is None:
            return               # frontier vanished mid-tick; try again
        dist_ok = True
        if self._goal_pose is not None:
            dist_ok = math.hypot(
                target[0] - self._goal_pose[0],
                target[1] - self._goal_pose[1]) >= GOAL_MIN_SPACING_M
        if dist_ok:
            self._goal_pose = target
            self._goals_sent += 1
            self._send_goal(*target)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()