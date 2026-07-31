#!/usr/bin/env python3
"""
battery_monitor.py
==================
Watches /battery_state and drives a simple 3-state machine:

    PATROLLING  →  GOING_TO_CHARGER  →  CHARGING  →  PATROLLING

State descriptions
------------------
PATROLLING
    Cycles through (x, y, yaw) waypoints via NavigateToPose.
    On goal completion the next waypoint is sent automatically.

GOING_TO_CHARGER
    Triggered when battery % drops below low_battery_threshold.
    Cancels the current Nav2 goal and sends the robot to the charger.

CHARGING
    Waits charge_duration seconds, calls /reset_battery, returns to PATROLLING.
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg     import OccupancyGrid
from sensor_msgs.msg  import BatteryState
from std_msgs.msg     import String
from std_srvs.srv     import Empty
from nav2_msgs.action import NavigateToPose
from action_msgs.msg  import GoalStatus


STATE_PATROLLING       = 'PATROLLING'
STATE_GOING_TO_CHARGER = 'GOING_TO_CHARGER'
STATE_CHARGING         = 'CHARGING'

# Minimum seconds between consecutive nav goal attempts (avoids rapid-fire loop)
RETRY_COOLDOWN = 3.0


def _yaw_to_q(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _make_pose(x, y, yaw, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    qx, qy, qz, qw = _yaw_to_q(float(yaw))
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p


class BatteryMonitor(Node):

    def __init__(self):
        super().__init__('battery_monitor')
        self._cbg = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('full_threshold',        95.0)
        self.declare_parameter('charge_duration',       15.0)
        self.declare_parameter('charger_x',              1.0)
        self.declare_parameter('charger_y',              1.0)
        self.declare_parameter('charger_yaw',            0.0)
        self.declare_parameter('patrol_waypoints', [
            2.5,  1.0,  0.0,
            2.5,  2.5,  1.5708,
            1.0,  2.5,  3.1416,
            1.0,  1.0, -1.5708,
        ])

        self._low_thresh  = self.get_parameter('low_battery_threshold').value
        self._charge_dur  = self.get_parameter('charge_duration').value
        self._charger_x   = self.get_parameter('charger_x').value
        self._charger_y   = self.get_parameter('charger_y').value
        self._charger_yaw = self.get_parameter('charger_yaw').value
        raw               = self.get_parameter('patrol_waypoints').value
        self._waypoints   = self._parse_wps(raw)

        # ── State ─────────────────────────────────────────────────────────────
        self._state          = STATE_PATROLLING
        self._battery_pct    = 100.0
        self._patrol_index   = 0
        self._goal_handle    = None          # protected by _goal_lock
        self._goal_in_flight = False
        self._charge_start   = None
        self._last_goal_time = 0.0           # cooldown between retries
        self._map_received   = False         # wait for SLAM map before sending goals
        self._goal_lock      = threading.Lock()

        # ── Action client ─────────────────────────────────────────────────────
        self._nav = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._cbg)

        # ── Service client ────────────────────────────────────────────────────
        self._reset_cli = self.create_client(
            Empty, '/reset_battery', callback_group=self._cbg)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            BatteryState, '/battery_state',
            self._battery_cb, 10, callback_group=self._cbg)

        # Wait for SLAM map — MUST use transient_local QoS to match slam_toolbox publisher.
        # Default (volatile) QoS never receives the map message.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            OccupancyGrid, '/map',
            self._map_cb, map_qos, callback_group=self._cbg)

        # ── Publisher ─────────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/robot_state', 10)

        # ── Main loop at 2 Hz (slower = less log spam) ────────────────────────
        self.create_timer(0.5, self._loop, callback_group=self._cbg)

        self.get_logger().info(
            f'BatteryMonitor ready  '
            f'low={self._low_thresh}%  dur={self._charge_dur}s  '
            f'{len(self._waypoints)} waypoints  '
            f'charger=({self._charger_x},{self._charger_y})\n'
            f'Waiting for SLAM map before sending first goal …')

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _parse_wps(self, flat):
        if len(flat) % 3:
            self.get_logger().error('patrol_waypoints must be triples (x, y, yaw)')
            return [(2.5, 1.0, 0.0), (2.5, 2.5, 1.5708),
                    (1.0, 2.5, 3.1416), (1.0, 1.0, -1.5708)]
        return [(flat[i], flat[i+1], flat[i+2]) for i in range(0, len(flat), 3)]

    def _pub_state(self):
        m = String()
        m.data = (f'STATE={self._state}  '
                  f'BAT={self._battery_pct:.1f}%  '
                  f'WP={self._patrol_index}  '
                  f'MAP={"YES" if self._map_received else "NO"}')
        self._state_pub.publish(m)

    # ── Subscribers ───────────────────────────────────────────────────────────

    def _battery_cb(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0

    def _map_cb(self, msg: OccupancyGrid):
        if not self._map_received and msg.info.width > 0 and msg.info.height > 0:
            self._map_received = True
            self.get_logger().info(
                f'SLAM map received ({msg.info.width}×{msg.info.height} cells) '
                f'— ready to send navigation goals.')

    # ── Nav2 goal management ──────────────────────────────────────────────────

    def _send_goal(self, x, y, yaw, label=''):
        now = time.time()
        if now - self._last_goal_time < RETRY_COOLDOWN:
            return   # still in cooldown, skip this tick
        self._last_goal_time = now

        if not self._nav.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose server not ready yet')
            return

        goal = NavigateToPose.Goal()
        goal.pose = _make_pose(x, y, yaw)
        goal.pose.header.stamp = self.get_clock().now().to_msg()

        self._goal_in_flight = True
        self.get_logger().info(
            f'→ Sending goal {label} ({x:.2f}, {y:.2f}, yaw={yaw:.2f})')
        fut = self._nav.send_goal_async(goal, feedback_callback=lambda _: None)
        fut.add_done_callback(self._goal_response_cb)

    def _goal_response_cb(self, future):
        # Use a LOCAL variable to avoid the race condition where _cancel_current_goal
        # sets self._goal_handle = None between the null-check and get_result_async().
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2 (map or planner not ready)')
            self._goal_in_flight = False
            with self._goal_lock:
                self._goal_handle = None
            return
        with self._goal_lock:
            self._goal_handle = goal_handle
        self.get_logger().info('Goal ACCEPTED')
        goal_handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        status = result.status
        self._goal_in_flight = False
        with self._goal_lock:
            self._goal_handle = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal SUCCEEDED ✓')
            self._on_succeeded()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal CANCELED')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn(
                'Goal ABORTED (Nav2 could not plan — map still building?). '
                f'Will retry in {RETRY_COOLDOWN}s.')
        else:
            self.get_logger().warn(f'Goal ended with status {status}')

    def _cancel_goal(self):
        with self._goal_lock:
            gh = self._goal_handle
            self._goal_handle = None
        self._goal_in_flight = False
        if gh is not None:
            self.get_logger().info('Cancelling active Nav2 goal …')
            gh.cancel_goal_async()

    # ── State transitions ─────────────────────────────────────────────────────

    def _on_succeeded(self):
        if self._state == STATE_PATROLLING:
            self._patrol_index = (self._patrol_index + 1) % len(self._waypoints)
            self.get_logger().info(f'Waypoint done → next WP index {self._patrol_index}')
        elif self._state == STATE_GOING_TO_CHARGER:
            self.get_logger().info('Arrived at charger → CHARGING')
            self._state        = STATE_CHARGING
            self._charge_start = time.time()

    def _go_to_charger(self):
        self.get_logger().warn(
            f'LOW BATTERY ({self._battery_pct:.1f}%) — cancelling patrol, heading to charger!')
        self._cancel_goal()
        self._state = STATE_GOING_TO_CHARGER
        self._last_goal_time = 0.0   # reset cooldown so charger goal sends immediately
        self._send_goal(self._charger_x, self._charger_y, self._charger_yaw, 'CHARGER')

    def _finish_charging(self):
        self.get_logger().info('Charging complete — resetting battery …')
        if self._reset_cli.wait_for_service(timeout_sec=2.0):
            self._reset_cli.call_async(Empty.Request()).add_done_callback(
                lambda _: self.get_logger().info('Battery reset to 100%'))
        else:
            self.get_logger().error('/reset_battery service unavailable')
        self._state = STATE_PATROLLING
        self._last_goal_time = 0.0
        self.get_logger().info(f'Resuming patrol from WP {self._patrol_index}')

    # ── Main loop ─────────────────────────────────────────────────────────────

    def _loop(self):
        self._pub_state()

        if self._state == STATE_PATROLLING:
            if self._battery_pct <= self._low_thresh:
                self._go_to_charger()
                return
            # Send next patrol goal if none in flight.
            # We no longer gate on _map_received — SLAM provides the map→odom
            # transform independently of the /map topic subscription.
            # RETRY_COOLDOWN prevents rapid-fire retries if Nav2 isn't ready yet.
            if not self._goal_in_flight:
                wp = self._waypoints[self._patrol_index]
                self._send_goal(wp[0], wp[1], wp[2], f'WP[{self._patrol_index}]')

        elif self._state == STATE_GOING_TO_CHARGER:
            if not self._goal_in_flight:
                # Retry after cooldown (avoids rapid-fire loop on ABORTED)
                self._send_goal(
                    self._charger_x, self._charger_y, self._charger_yaw,
                    'CHARGER (retry)')

        elif self._state == STATE_CHARGING:
            elapsed = time.time() - self._charge_start
            self.get_logger().info(
                f'Charging {elapsed:.0f}/{self._charge_dur:.0f}s  '
                f'bat={self._battery_pct:.1f}%',
                throttle_duration_sec=3.0)
            if elapsed >= self._charge_dur:
                self._finish_charging()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
