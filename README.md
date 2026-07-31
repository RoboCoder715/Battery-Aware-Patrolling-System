# Battery-Aware Patrol Robot

A ROS 2 (Humble) demonstration package that simulates a **draining battery** on a TurtleBot3 performing **continuous waypoint patrol**. When the battery drops below a threshold, a monitor node cancels the active Nav2 goal and redirects the robot to a **charging station**. After a simulated recharge, patrol resumes automatically.

> **Learning goals:** ROS 2 pub/sub, action clients, service calls, lifecycle awareness, and simple if/else state machine — no SMACH needed.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  battery_simulator  ──/battery_state──►  battery_monitor    │
│                                               │             │
│                          ┌────────────────────┤             │
│                          │  State Machine      │             │
│                          │  PATROLLING         │             │
│                          │  GOING_TO_CHARGER   │             │
│                          │  CHARGING           │             │
│                          └────────────────────┘             │
│                               │  NavigateToPose Action      │
│                               ▼                             │
│                        Nav2  (slam_toolbox + planner        │
│                               + controller + BT navigator)  │
│                               │                             │
│                               ▼                             │
│                     TurtleBot3 burger  (Gazebo)             │
└─────────────────────────────────────────────────────────────┘
```

### State Machine

```
         ┌──────────────────────────────────────┐
    ┌───►│          PATROLLING                  │◄──── after charge
    │    │  Cycles WP[0]→WP[1]→WP[2]→WP[3]→…  │
    │    └──────────────┬───────────────────────┘
    │                   │  battery < 20 %
    │                   ▼
    │    ┌──────────────────────────────────────┐
    │    │       GOING_TO_CHARGER               │
    │    │  Cancel patrol goal → send robot     │
    │    │  to fixed charging-station pose      │
    │    └──────────────┬───────────────────────┘
    │                   │  arrived at charger
    │                   ▼
    │    ┌──────────────────────────────────────┐
    └────│          CHARGING                    │
         │  Wait 15 s → call /reset_battery     │
         │  → battery jumps to 100 %            │
         └──────────────────────────────────────┘
```

---

## Package Structure

```
battery_patrol/
├── battery_patrol/
│   ├── battery_simulator.py   # Drains battery, publishes /battery_state
│   └── battery_monitor.py     # State machine + Nav2 action client
├── config/
│   ├── patrol_params.yaml     # Waypoints, thresholds, charger position
│   └── nav2_params.yaml       # Custom Nav2 config (rolling costmap)
├── launch/
│   └── battery_patrol.launch.py
├── package.xml
├── setup.py
└── README.md
```

---

## Prerequisites

**ROS 2 Humble** with the following packages:

```bash
sudo apt install -y \
  ros-humble-turtlebot3 \
  ros-humble-turtlebot3-gazebo \
  ros-humble-turtlebot3-simulations \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox
```

---

## Build

```bash
cd ~/project5_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select battery_patrol --symlink-install
source install/setup.bash
```

---

## Run

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source install/setup.bash

# Full stack (Gazebo + SLAM + Nav2 + battery nodes)
ros2 launch battery_patrol battery_patrol.launch.py

# Faster demo — battery drains 4× quicker
ros2 launch battery_patrol battery_patrol.launch.py drain_rate:=4.0

# Battery nodes only (if Gazebo + Nav2 already running)
ros2 launch battery_patrol battery_patrol.launch.py nodes_only:=true
```

> The battery nodes start **25 seconds after launch** to allow Gazebo, SLAM Toolbox, and Nav2 to fully initialise before the first navigation goal is sent.

---

## Visualise

Open new terminals (after sourcing ROS 2):

```bash
# State machine — PATROLLING / GOING_TO_CHARGER / CHARGING
ros2 topic echo /robot_state

# Battery level (0.0–1.0, multiply by 100 for %)
ros2 topic echo /battery_state --field percentage

# Human-readable battery string "Battery: 73.4%"
ros2 topic echo /battery_sim_status

# Live plot of battery drain + recharge jump
ros2 run rqt_plot rqt_plot /battery_state/percentage
```

---

## Nodes

### `battery_simulator`

| Item | Detail |
|---|---|
| **Publishes** | `/battery_state` (`sensor_msgs/BatteryState`) |
| **Publishes** | `/battery_sim_status` (`std_msgs/String`) |
| **Service** | `/reset_battery` (`std_srvs/Empty`) — jumps battery to 100 % |
| **Key param** | `drain_rate` — % per second (default `1.0`) |

Decreases an internal `float` counter each timer tick. Clamps to `[0, 100]`.  
The `/reset_battery` service instantly restores the counter to 100 %.

### `battery_monitor`

| Item | Detail |
|---|---|
| **Subscribes** | `/battery_state` |
| **Action client** | `navigate_to_pose` (Nav2) |
| **Service client** | `/reset_battery` |
| **Publishes** | `/robot_state` (`std_msgs/String`) |

Runs at 2 Hz. Contains the if/else state machine. Key safety features:
- **Race-condition fix** — uses a local variable + `threading.Lock` for the goal handle so the timer thread cannot nullify it mid-callback.
- **Retry cooldown** — minimum 3 s between consecutive navigation goal attempts to prevent rapid-fire loops when Nav2 is busy.
- **SLAM QoS** — subscribes to `/map` with `transient_local` QoS to correctly receive SLAM Toolbox map messages.

---

## Configuration

All parameters live in `config/patrol_params.yaml`:

| Parameter | Default | Description |
|---|---|---|
| `drain_rate` | `1.0` | % drained per second |
| `low_battery_threshold` | `20.0` | % that triggers go-to-charger |
| `charge_duration` | `15.0` | Seconds to wait at charger |
| `charger_x/y/yaw` | `1.0, 1.0, 0.0` | Charging station pose (map frame) |
| `patrol_waypoints` | 4-corner square | Flat list of `x y yaw` triples |

### Default Patrol Route

The robot spawns at `(1.0, 1.0)` and loops a square:

```
WP0 (2.5, 1.0) ──► WP1 (2.5, 2.5)
     ▲                    │
     │                    ▼
WP3 (1.0, 1.0) ◄── WP2 (1.0, 2.5)
```

---

## Nav2 Configuration Notes

The package ships its own `config/nav2_params.yaml` with one critical difference from the nav2_bringup default:

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      rolling_window: true          # ← No static map layer needed
      plugins: ["obstacle_layer", "inflation_layer"]
```

Using a **rolling global costmap** (no `static_layer`) avoids the `"Received map message is malformed"` error that occurs when SLAM Toolbox publishes its initial, partially-filled map. SLAM Toolbox still runs and provides the `map → odom → base_link` transform chain needed for global localisation.

---

## Manual Controls

```bash
# Manually trigger low-battery event (for testing)
ros2 service call /reset_battery std_srvs/srv/Empty {}   # resets to 100%

# Cancel current Nav2 goal
ros2 action cancel_goal /navigate_to_pose <goal_id>

# Teleop override (stops autonomous patrol)
ros2 run turtlebot3_teleop teleop_keyboard
```

---

## Student Extensions

| Idea | Hint |
|---|---|
| **Distance-based drain** | Subscribe to `/odom`, accumulate `Δpose`, drain proportional to metres travelled |
| **Gradual recharge** | Instead of instant reset, increment battery % each tick while in `CHARGING` state |
| **Multiple chargers** | Store a list of charger poses; pick the nearest using Euclidean distance to `/odom` |
| **Resume from interruption** | Record the exact robot pose when alarm triggers; return there after charging instead of next waypoint |
| **RViz battery panel** | Publish a `visualization_msgs/Marker` or use `rqt_plot` to display battery bar |
| **Email / sound alert** | Publish to a `/low_battery_alert` topic and subscribe from a notifier node |
