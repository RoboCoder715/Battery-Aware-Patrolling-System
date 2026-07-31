#!/usr/bin/env python3
"""
battery_patrol.launch.py  —  Battery-aware patrol demo
=======================================================
Starts:
  1. TurtleBot3 Gazebo (empty world) — confirmed working, correct TF tree
  2. Nav2 bringup with SLAM Toolbox  — slam:=True, no pre-built map needed
  3. battery_simulator               — drains battery over time
  4. battery_monitor                 — state machine, cancels Nav2 goal on low battery

Usage
-----
  export TURTLEBOT3_MODEL=burger
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch battery_patrol battery_patrol.launch.py drain_rate:=4.0

Nodes-only (Gazebo + Nav2 already running in another terminal)
--------------------------------------------------------------
  ros2 launch battery_patrol battery_patrol.launch.py nodes_only:=true

Visualise battery
-----------------
  ros2 topic echo /battery_sim_status          # "Battery: 42.3%"
  ros2 topic echo /battery_state --field percentage   # 0.0–1.0
  ros2 topic echo /robot_state                 # PATROLLING / GOING_TO_CHARGER / CHARGING
  ros2 run rqt_plot rqt_plot /battery_state/percentage  # live graph
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
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
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)   # plain string
    drain_rate   = LaunchConfiguration('drain_rate').perform(context)     # plain string

    params_yaml  = os.path.join(_pkg('battery_patrol'), 'config', 'patrol_params.yaml')
    actions      = []

    # ── 1. Simulation stack ────────────────────────────────────────────────────
    if not nodes_only:
        missing = [p for p in ('turtlebot3_gazebo', 'nav2_bringup', 'slam_toolbox')
                   if not _pkg(p)]
        if missing:
            print('\nMissing packages: ' + ', '.join(missing), file=sys.stderr)
            print('sudo apt install -y ' +
                  ' '.join(f'ros-humble-{p.replace("_", "-")}' for p in missing),
                  file=sys.stderr)
            sys.exit(1)

        # All paths resolved as plain Python strings inside OpaqueFunction
        # so RewrittenYaml never receives an empty LaunchConfiguration.
        tb3_gazebo_dir   = _pkg('turtlebot3_gazebo')
        nav2_bringup_dir = _pkg('nav2_bringup')
        nav2_params      = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')

        # 1a. TurtleBot3 Gazebo — empty world, spawns burger, publishes full TF tree
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_gazebo_dir, 'launch', 'empty_world.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ))

        # 1b. Nav2 bringup with SLAM (slam:=True → SLAM Toolbox replaces AMCL).
        #     bringup_launch.py requires 'map' even in SLAM mode (declared but
        #     not loaded at runtime when slam:=True). Point to the bundled map.
        #     params_file is an explicit string → RewrittenYaml never gets ''.
        dummy_map = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam':         'True',
                'use_sim_time': use_sim_time,
                'params_file':  nav2_params,
                'map':          dummy_map,
            }.items(),
        ))

    # ── 2. battery_simulator ───────────────────────────────────────────────────
    actions.append(Node(
        package='battery_patrol',
        executable='battery_simulator',
        name='battery_simulator',
        output='screen',
        parameters=[params_yaml, {
            'use_sim_time': use_sim_time == 'true',
            'drain_rate': float(drain_rate),
        }],
    ))

    # ── 3. battery_monitor — delayed so Nav2 finishes initialising ─────────────
    actions.append(TimerAction(
        period=10.0 if not nodes_only else 2.0,
        actions=[Node(
            package='battery_patrol',
            executable='battery_monitor',
            name='battery_monitor',
            output='screen',
            parameters=[params_yaml, {
                'use_sim_time': use_sim_time == 'true',
            }],
        )],
    ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo simulation clock'),
        DeclareLaunchArgument('drain_rate', default_value='1.0',
                              description='Battery drain %% per second (try 4.0 for quick demo)'),
        DeclareLaunchArgument('nodes_only', default_value='false',
                              description='Skip Gazebo+Nav2; only start battery nodes'),
        OpaqueFunction(function=_launch_setup),
    ])
