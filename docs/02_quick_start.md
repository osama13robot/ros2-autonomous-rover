# 02 · Quick start

## Start everything

```bash
cd ~/ros2-autonomous-rover
./scripts/start_sim.sh
```

This starts Gazebo, RViz and the whole autonomy stack, and opens a second terminal window with the
keyboard control (after about 8 s). Without `gnome-terminal`, open a new terminal yourself and run:

```bash
ros2 run rover_control teleop_keyboard
```

> The keyboard node needs a **real interactive terminal**, so it can't run inside a launch file.

Useful variations (any `name:=value` is passed to the launch file):

```bash
./scripts/start_sim.sh headless:=true          # no Gazebo window (much faster), RViz only
./scripts/start_sim.sh initial_mode:=auto      # start exploring immediately
./scripts/start_sim.sh world:=empty.sdf        # flat empty world
./scripts/start_sim.sh robot:=rover4           # the 4WD car-like robot (keyboard in car mode)
```

## What you see

**Gazebo** shows the physical simulation: the robot in a 14 × 10 m apartment.

**RViz** shows what the robot *knows*:

| Display | Meaning |
|---|---|
| grey/black map | the SLAM map being built (black = wall, grey = unknown) |
| red dots | lidar hits |
| coloured points | the depth camera's 3D point cloud |
| blue/purple haze | costmaps: where the robot should not go |
| blue line | the planned path |
| green dots / blue spheres / red cylinder | frontier cells / candidate goals / current exploration target |
| camera panel | the RGB camera image |

## Drive it (MANUAL mode)

```
   q  w  e        hold a key to drive; release it and the robot stops (~0.6 s)
   a  s  d        w/x forward/back · a/d turn · q/e/z/c curves · s or SPACE stop
   z  x  c        arrow keys work too
   r/f  speed ±10 %     t/g  turn speed ±10 %
```

The status line at the bottom shows the mode, the speed, and the explorer's status.

## Go autonomous (AUTO mode)

Press **`2`** (or `m` to toggle). The robot will:

1. pick the most attractive **frontier** (border of unknown space),
2. plan a path and drive there, avoiding obstacles,
3. repeat until nothing unknown is reachable,
4. drive back to where it started, and report `DONE`.

Take over at any moment by pressing **any drive key**; the robot switches to MANUAL and cancels
its goal. Press `2` to hand control back. `o` pauses or resumes exploration.

### Send your own goal

In AUTO, click **2D Goal Pose** in the RViz toolbar, then click and drag on the map. The explorer
pauses (your goal wins); press `o` to resume exploring afterwards.

## Save the map, then navigate on it

When exploration reports `DONE`:

```bash
./scripts/save_map.sh my_apartment
# → src/rover_navigation/maps/my_apartment.yaml + .pgm
```

Next time, skip mapping and localize on the saved map with **AMCL**:

```bash
./scripts/start_sim.sh slam:=false map:=$HOME/ros2-autonomous-rover/src/rover_navigation/maps/my_apartment.yaml
```

A ready-made map of the default world is included, so you can try this immediately:

```bash
./scripts/start_sim.sh slam:=false map:=$HOME/ros2-autonomous-rover/src/rover_navigation/maps/rover_world.yaml
```

Then run the patrol example, which visits 5 rooms in turn:

```bash
ros2 run rover_control waypoint_patrol --ros-args -p use_sim_time:=true
```

## Control from the command line

Everything the keyboard does is also available as ROS interfaces:

```bash
ros2 topic pub --once /mode_request std_msgs/String "{data: auto}"   # manual | auto | toggle
ros2 service call /set_autonomous std_srvs/srv/SetBool "{data: false}"
ros2 service call /explore/enable std_srvs/srv/SetBool "{data: true}"
ros2 topic echo /robot_mode
ros2 topic echo /explore/status
```

## All launch arguments

`ros2 launch rover_bringup sim.launch.py <arg>:=<value>`:

| Argument | Default | Meaning |
|---|---|---|
| `robot` | `rover` | `rover` (2WD, turns in place) or `rover4` (4WD Ackermann, see [14](14_rover4_ackermann.md)) |
| `world` | `rover_world.sdf` | a world file from `rover_gazebo/worlds` |
| `headless` | `false` | run Gazebo without its GUI |
| `x`, `y`, `yaw` | `0.0` | spawn pose |
| `slam` | `true` | `true` = build a map; `false` = AMCL on `map` |
| `map` | `''` | map yaml (only with `slam:=false`) |
| `explore` | `true` | start exploring automatically when switched to AUTO |
| `initial_mode` | `manual` | `manual` or `auto` |
| `rviz` | `true` | start RViz |

## Stopping

Press **Ctrl-C** in the launch terminal. If Gazebo is still running afterwards (`pgrep -f "gz sim"`),
stop it with `pkill -f 'gz sim'`: a leftover simulator confuses the next run. `start_sim.sh`
warns you about this.
