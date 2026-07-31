#!/usr/bin/env python3
"""
battery_simulator.py
====================
Simulates a robot battery that drains over time and publishes the level on
/battery_state (sensor_msgs/BatteryState).

Topics published
----------------
  /battery_state  (sensor_msgs/BatteryState)   – battery percentage [0-100]
  /robot_state    (std_msgs/String)             – human-readable state string

Services provided
-----------------
  /reset_battery  (std_srvs/srv/Empty)          – resets battery to 100 %

Parameters
----------
  drain_rate      (float, default 1.0)  – % drained per second
  initial_charge  (float, default 100.0)
  publish_rate    (float, default 1.0)  – Hz

Student exercise hint
---------------------
  Change 'drain_rate' to make the battery drain faster or slower.
  You could also subscribe to /odom and drain proportional to distance
  traveled instead of time.
"""

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from std_srvs.srv import Empty


class BatterySimulator(Node):
    """Publishes a synthetic, draining battery percentage."""

    def __init__(self):
        super().__init__('battery_simulator')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('drain_rate',     1.0)   # % per second
        self.declare_parameter('initial_charge', 100.0)
        self.declare_parameter('publish_rate',   1.0)   # Hz

        self._drain_rate     = self.get_parameter('drain_rate').value
        self._charge         = self.get_parameter('initial_charge').value
        publish_rate         = self.get_parameter('publish_rate').value

        # ── Publishers ────────────────────────────────────────────────────────
        self._battery_pub = self.create_publisher(
            BatteryState, '/battery_state', 10)
        self._state_pub   = self.create_publisher(
            String, '/battery_sim_status', 10)

        # ── Service ───────────────────────────────────────────────────────────
        self._reset_srv = self.create_service(
            Empty, '/reset_battery', self._reset_callback)

        # ── Timer ─────────────────────────────────────────────────────────────
        period = 1.0 / publish_rate
        self._timer = self.create_timer(period, self._timer_callback)

        self.get_logger().info(
            f'BatterySimulator started  '
            f'(drain={self._drain_rate}%/s, '
            f'init={self._charge}%, '
            f'rate={publish_rate}Hz)')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _timer_callback(self):
        """Drain battery and publish."""
        # Drain by drain_rate % per timer tick
        # (timer period = 1/publish_rate seconds)
        dt = 1.0 / self.get_parameter('publish_rate').value
        self._charge = max(0.0, self._charge - self._drain_rate * dt)

        # Build BatteryState message
        msg = BatteryState()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.percentage      = self._charge / 100.0   # ROS convention: 0.0–1.0
        msg.power_supply_status = (
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            if self._charge > 0.0
            else BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        )
        self._battery_pub.publish(msg)

        # Human-readable status
        status = String()
        status.data = f'Battery: {self._charge:.1f}%'
        self._state_pub.publish(status)

        self.get_logger().debug(f'Battery: {self._charge:.1f}%')

    def _reset_callback(self, _request, response):
        """Reset battery to 100% (called by battery_monitor after charging)."""
        self._charge = 100.0
        self.get_logger().info('Battery RESET to 100%')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
