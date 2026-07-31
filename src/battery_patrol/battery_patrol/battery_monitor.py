#!/usr/bin/env python3
"""
battery_monitor.py
==================
Watches /battery_state and drives a simple 3-state machine:

    PATROLLING  →  GOING_TO_CHARGER  →  CHARGING  →  PATROLLING

State descriptions
------------------
PATROLLING
    The robot cycles through a list of (x, y, yaw) waypoints using the
    Nav2 NavigateToPose action.  When the current goal finishes the next
    waypoint is sent automatically.

GOING_TO_CHARGER
    Triggered when battery % drops below `low_battery_threshold`.
    Cancels the current Nav2 goal and sends the robot to a fixed
    charging-station pose.

CHARGING
    Robot is at the charger.  Waits `charge_duration` seconds then calls
    /reset_battery to restore power, and transitions back to PATROLLING.

Topics subscribed
-----------------
  /battery_state   (sensor_msgs/BatteryState)

Topics published
----------------
  /robot_state     (std_msgs/String)   – current state for visualisation

Action client
-------------
  navigate_to_pose  (nav2_msgs/action/NavigateToPose)

Service client
--------------
  /reset_battery   (std_srvs/srv/Empty)

Parameters
----------
  low_battery_threshold  (float, default 20.0)  – % that triggers go-to-charger
  full_threshold         (float, default 95.0)  – % at which charging is done
  charge_duration        (float, default 15.0)  – seconds to wait at charger
  charger_x              (float, default 0.0)
  charger_y              (float, default 0.0)
  charger_yaw            (float, default 0.0)
  patrol_waypoints       (list[float], default [])
      Flat list: x0 y0 yaw0  x1 y1 yaw1 …
      Set via YAML or --ros-args -p patrol_waypoints:=[…]
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg  import BatteryState
from std_msgs.msg     import String
from std_srvs.srv     import Empty
from nav2_msgs.action import NavigateToPose
from action_msgs.msg  import GoalStatus


# ── State constants ────────────────────────────────────────────────────────────
STATE_PATROLLING        = 'PATROLLING'
STATE_GOING_TO_CHARGER  = 'GOING_TO_CHARGER'
STATE_CHARGING          = 'CHARGING'


def yaw_to_quaternion(yaw: float):
    """Convert a yaw angle (rad) to a geometry_msgs-compatible quaternion dict."""
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0),
    }


def make_pose_stamped(x: float, y: float, yaw: float, frame: str = 'map') -> PoseStamped:
    """Build a PoseStamped from (x, y, yaw)."""
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    q = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = q['x']
    pose.pose.orientation.y = q['y']
    pose.pose.orientation.z = q['z']
    pose.pose.orientation.w = q['w']
    return pose


class BatteryMonitor(Node):
    """State machine node that supervises patrol and handles low-battery events."""

    def __init__(self):
        super().__init__('battery_monitor')

        # ── Callback group (allows service calls inside timer / sub callbacks) ─
        self._cbg = ReentrantCallbackGroup()

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('full_threshold',        95.0)
        self.declare_parameter('charge_duration',       15.0)
        self.declare_parameter('charger_x',              0.0)
        self.declare_parameter('charger_y',              0.0)
        self.declare_parameter('charger_yaw',            0.0)
        # Flat list: x0 y0 yaw0  x1 y1 yaw1 …
        self.declare_parameter('patrol_waypoints', [
            1.5,  0.0,  0.0,
            1.5,  1.5,  1.5708,
            0.0,  1.5,  3.1416,
            0.0,  0.0, -1.5708,
        ])

        self._low_thresh     = self.get_parameter('low_battery_threshold').value
        self._full_thresh    = self.get_parameter('full_threshold').value
        self._charge_dur     = self.get_parameter('charge_duration').value
        self._charger_x      = self.get_parameter('charger_x').value
        self._charger_y      = self.get_parameter('charger_y').value
        self._charger_yaw    = self.get_parameter('charger_yaw').value

        raw_wps = self.get_parameter('patrol_waypoints').value
        self._waypoints = self._parse_waypoints(raw_wps)

        # ── State ─────────────────────────────────────────────────────────────
        self._state          = STATE_PATROLLING
        self._battery_pct    = 100.0
        self._patrol_index   = 0          # current waypoint index
        self._goal_handle    = None       # active Nav2 goal handle
        self._goal_in_flight = False      # True while a goal is being sent/active
        self._charge_start   = None       # timestamp when charging began

        # ── Action client ─────────────────────────────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cbg,
        )

        # ── Service client ─────────────────────────────────────────────────────
        self._reset_client = self.create_client(
            Empty, '/reset_battery',
            callback_group=self._cbg,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self._battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self._battery_callback,
            10,
            callback_group=self._cbg,
        )

        # ── Publisher ─────────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/robot_state', 10)

        # ── Main loop timer (5 Hz) ────────────────────────────────────────────
        self._loop_timer = self.create_timer(
            0.2, self._loop, callback_group=self._cbg)

        self.get_logger().info(
            f'BatteryMonitor ready  '
            f'(low={self._low_thresh}%, full={self._full_thresh}%, '
            f'charge_dur={self._charge_dur}s, '
            f'{len(self._waypoints)} waypoints)')

    # ── Helper ────────────────────────────────────────────────────────────────

    def _parse_waypoints(self, flat: list):
        """Convert flat [x0,y0,yaw0, x1,y1,yaw1, …] to [(x,y,yaw), …]."""
        if len(flat) % 3 != 0:
            self.get_logger().error(
                'patrol_waypoints length must be a multiple of 3 (x, y, yaw). '
                f'Got {len(flat)} values — using defaults.')
            return [(1.5, 0.0, 0.0), (1.5, 1.5, 1.5708),
                    (0.0, 1.5, 3.1416), (0.0, 0.0, -1.5708)]
        return [(flat[i], flat[i+1], flat[i+2]) for i in range(0, len(flat), 3)]

    def _publish_state(self):
        msg = String()
        msg.data = (f'STATE={self._state}  '
                    f'BATTERY={self._battery_pct:.1f}%  '
                    f'WP={self._patrol_index}')
        self._state_pub.publish(msg)

    # ── Battery subscriber ────────────────────────────────────────────────────

    def _battery_callback(self, msg: BatteryState):
        self._battery_pct = msg.percentage * 100.0  # convert 0-1 → 0-100

    # ── Nav2 goal management ──────────────────────────────────────────────────

    def _send_goal(self, x: float, y: float, yaw: float, label: str = ''):
        """Send a NavigateToPose goal and register result/feedback callbacks."""
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = make_pose_stamped(x, y, yaw)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        self._goal_in_flight = True
        self.get_logger().info(
            f'Sending goal {label}  →  ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)')

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_callback,
        )
        send_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle or not self._goal_handle.accepted:
            self.get_logger().warn('Goal REJECTED by Nav2')
            self._goal_in_flight = False
            return
        self.get_logger().info('Goal ACCEPTED')
        result_future = self._goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        result = future.result()
        status = result.status
        self._goal_in_flight = False
        self._goal_handle    = None

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal SUCCEEDED')
            self._on_goal_succeeded()
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal CANCELED')
        else:
            self.get_logger().warn(f'Goal ended with status {status}')

    def _feedback_callback(self, feedback_msg):
        # Uncomment to see navigation distance remaining:
        # remaining = feedback_msg.feedback.distance_remaining
        # self.get_logger().debug(f'Distance remaining: {remaining:.2f}m')
        pass

    def _cancel_current_goal(self):
        """Cancel any active Nav2 goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Canceling current Nav2 goal …')
            cancel_future = self._goal_handle.cancel_goal_async()
            # Fire-and-forget; _goal_result_callback will clear _goal_in_flight
        self._goal_in_flight = False
        self._goal_handle    = None

    # ── State transitions ─────────────────────────────────────────────────────

    def _on_goal_succeeded(self):
        """Called when any Nav2 goal completes successfully."""
        if self._state == STATE_PATROLLING:
            # Advance to next waypoint
            self._patrol_index = (self._patrol_index + 1) % len(self._waypoints)
            self.get_logger().info(
                f'Waypoint reached → next patrol index: {self._patrol_index}')

        elif self._state == STATE_GOING_TO_CHARGER:
            # Arrived at charger → start charging
            self.get_logger().info('Arrived at charger → CHARGING')
            self._state        = STATE_CHARGING
            self._charge_start = time.time()

    def _transition_to_charger(self):
        """Low battery detected → cancel patrol, head to charger."""
        self.get_logger().warn(
            f'LOW BATTERY ({self._battery_pct:.1f}%) '
            f'— abandoning patrol and going to charger!')
        self._cancel_current_goal()
        self._state = STATE_GOING_TO_CHARGER
        self._send_goal(
            self._charger_x, self._charger_y, self._charger_yaw,
            label='CHARGER')

    def _finish_charging(self):
        """Charging complete → reset battery, resume patrol."""
        self.get_logger().info('Charging complete → calling /reset_battery …')

        # Call the reset service (synchronous-style via wait)
        if self._reset_client.wait_for_service(timeout_sec=3.0):
            req = Empty.Request()
            future = self._reset_client.call_async(req)
            # We don't block here; the loop will detect battery > full_thresh
            future.add_done_callback(
                lambda _: self.get_logger().info('Battery reset confirmed'))
        else:
            self.get_logger().error('/reset_battery service not available!')

        self._state = STATE_PATROLLING
        self.get_logger().info(
            f'Resuming patrol from waypoint index {self._patrol_index}')

    # ── Main state-machine loop ────────────────────────────────────────────────

    def _loop(self):
        """Runs at 5 Hz. Central if/else state machine."""
        self._publish_state()

        # ── PATROLLING ────────────────────────────────────────────────────────
        if self._state == STATE_PATROLLING:
            # Check for low battery first
            if self._battery_pct <= self._low_thresh:
                self._transition_to_charger()
                return

            # If no goal is running, send the next patrol waypoint
            if not self._goal_in_flight:
                wp = self._waypoints[self._patrol_index]
                self._send_goal(
                    wp[0], wp[1], wp[2],
                    label=f'WP[{self._patrol_index}]')

        # ── GOING_TO_CHARGER ──────────────────────────────────────────────────
        elif self._state == STATE_GOING_TO_CHARGER:
            # If goal was dropped for some reason, resend it
            if not self._goal_in_flight:
                self.get_logger().warn(
                    'No active goal while GOING_TO_CHARGER — resending …')
                self._send_goal(
                    self._charger_x, self._charger_y, self._charger_yaw,
                    label='CHARGER (retry)')

        # ── CHARGING ──────────────────────────────────────────────────────────
        elif self._state == STATE_CHARGING:
            elapsed = time.time() - self._charge_start
            remaining = self._charge_dur - elapsed
            self.get_logger().info(
                f'Charging … {elapsed:.1f}/{self._charge_dur:.1f}s  '
                f'(battery={self._battery_pct:.1f}%)',
                throttle_duration_sec=2.0)

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
