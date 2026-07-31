#!/usr/bin/env python3
"""
battery_patrol.launch.py
========================
Starts:
  1. TurtleBot3 Gazebo empty world (burger at 1.0, 1.0)
  2. SLAM Toolbox async  — provides map→odom transform (localization)
  3. Nav2 navigation     — planner + controller + BT navigator
                           uses rolling global costmap (no static map needed)
  4. battery_simulator   — drains battery; starts after Nav2 is ready
  5. battery_monitor     — state machine; starts with battery_simulator

Usage
-----
  export TURTLEBOT3_MODEL=burger
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch battery_patrol battery_patrol.launch.py drain_rate:=4.0

Nodes-only (sim already running)
---------------------------------
  ros2 launch battery_patrol battery_patrol.launch.py nodes_only:=true

Monitor
-------
  ros2 topic echo /robot_state
  ros2 topic echo /battery_sim_status
  ros2 run rqt_plot rqt_plot /battery_state/percentage
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def _pkg(name):
    try:
        return get_package_share_directory(name)
    except Exception:
        return ''


def _launch_setup(context, *args, **kwargs):
    from launch.substitutions import LaunchConfiguration

    nodes_only   = LaunchConfiguration('nodes_only').perform(context) == 'true'
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    drain_rate   = LaunchConfiguration('drain_rate').perform(context)

    pkg_dir      = _pkg('battery_patrol')
    patrol_yaml  = os.path.join(pkg_dir, 'config', 'patrol_params.yaml')
    nav2_yaml    = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    actions      = []

    # ── Simulation stack ───────────────────────────────────────────────────────
    if not nodes_only:
        required = ['turtlebot3_gazebo', 'nav2_bringup', 'slam_toolbox']
        missing  = [p for p in required if not _pkg(p)]
        if missing:
            print('\nMissing: ' + ', '.join(missing), file=sys.stderr)
            print('sudo apt install -y ' +
                  ' '.join(f'ros-humble-{p.replace("_","-")}' for p in missing),
                  file=sys.stderr)
            sys.exit(1)

        tb3_dir  = _pkg('turtlebot3_gazebo')
        nav2_dir = _pkg('nav2_bringup')
        slam_dir = _pkg('slam_toolbox')

        # 1. Gazebo + TurtleBot3 burger — confirmed working TF tree
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_dir, 'launch', 'empty_world.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': '1.0',
                'y_pose': '1.0',
            }.items(),
        ))

        # 2. SLAM Toolbox async — provides map→odom localization transform.
        #    Using online_async (not sync) for more stable map publishing.
        slam_launch = os.path.join(slam_dir, 'launch', 'online_async_launch.py')
        if not os.path.exists(slam_launch):
            slam_launch = os.path.join(slam_dir, 'launch', 'online_sync_launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': nav2_yaml,
            }.items(),
        ))

        # 3. Nav2 navigation stack — planner + controller + BT navigator.
        #    Uses our custom nav2_params.yaml which has:
        #      - rolling global costmap (no static_layer → no malformed-map crash)
        #      - standard local costmap with obstacle layer
        #    params_file passed as plain string (avoids RewrittenYaml empty-path crash).
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file':  nav2_yaml,
            }.items(),
        ))

    # ── Battery nodes (both delayed so Nav2 + SLAM are ready) ─────────────────
    delay = 25.0 if not nodes_only else 2.0

    actions.append(TimerAction(
        period=delay,
        actions=[Node(
            package='battery_patrol',
            executable='battery_simulator',
            name='battery_simulator',
            output='screen',
            parameters=[patrol_yaml, {
                'use_sim_time': use_sim_time == 'true',
                'drain_rate': float(drain_rate),
            }],
        )],
    ))

    actions.append(TimerAction(
        period=delay,
        actions=[Node(
            package='battery_patrol',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            parameters=[patrol_yaml, {
                'use_sim_time': use_sim_time == 'true',
            }],
        )],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('drain_rate',   default_value='1.0',
                              description='%% per second (4.0 for quick demo)'),
        DeclareLaunchArgument('nodes_only',   default_value='false',
                              description='Skip Gazebo+Nav2'),
        OpaqueFunction(function=_launch_setup),
    ])
