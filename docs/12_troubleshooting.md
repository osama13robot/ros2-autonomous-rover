# 12 · Troubleshooting

## Start-up

| Symptom | Cause and fix |
|---|---|
| `Package 'rover_bringup' not found` | the workspace isn't sourced: `source install/setup.bash` |
| `colcon build` fails with a missing package | run `rosdep install --from-paths src --ignore-src -y` |
| The keyboard window doesn't open | `start_sim.sh` needs `gnome-terminal`; otherwise run `ros2 run rover_control teleop_keyboard` yourself |
| `teleop_keyboard needs an interactive terminal` | it can't run inside `ros2 launch` or a pipe; use a normal terminal |
| `WARNING: a Gazebo server is already running` | a previous run didn't shut down cleanly: `pkill -f 'gz sim'` |
| The robot behaves strangely, sensors jump around, "ghost" obstacles | **two simulators are running at once** and both publish the same topics. Stop everything: `pkill -f 'gz sim'`, then check `pgrep -af "gz sim"` |
| Gazebo shows the robot but RViz shows nothing | RViz Fixed Frame is `map`, and `map` exists only once SLAM has started (a few seconds) |

## Performance

| Symptom | Fix |
|---|---|
| Gazebo real-time factor well below 1.0 | `headless:=true`; close other programs; see [05 Simulation](05_simulation.md#performance-tips) |
| `Control loop missed its desired rate` warnings | normal on slow machines; reduce MPPI `batch_size` if the robot drives badly |
| `collision_monitor: … timestamps differ … Ignoring the source` followed by `Robot to stop` | the camera data arrived more than `source_timeout` late, so the robot pauses for safety. Normal on a slow simulator; raise `source_timeout` or reduce the simulator load |
| `map_saver` fails with a timeout | the simulator is slow; `save_map.sh` already uses 20 s; pass `-p save_map_timeout:=60.0` if needed |

## Driving and navigation

| Symptom | Cause and fix |
|---|---|
| The keyboard does nothing | the robot is in AUTO (status line shows `[AUTO]`). Press `1`, or any drive key to take over |
| Goals from RViz are ignored | the robot is in MANUAL. Press `2` first |
| AUTO, but the robot doesn't move | check `ros2 topic echo /explore/status`. `WAITING for Nav2`: Nav2 is still starting (about 10 s). `DONE`: nothing left to explore |
| The robot stops and starts a lot near obstacles | the collision monitor is being cautious; lower `time_before_collision` |
| `Start occupied` / `Failed to create plan` | the robot or the goal is inside an inflated obstacle. Pick goals further from walls, or reduce `inflation_radius` |
| `Failed to make progress` then spins or backs up | recoveries at work; normal occasionally. Often, the goal is in a narrow gap |
| The explorer leaves grey patches | areas it cannot see into (behind furniture) are skipped on purpose; see [09](09_autonomous_exploration.md) |
| AMCL: the robot is in the wrong place on the map | use *2D Pose Estimate* in RViz to give the right start pose |
| `ros2 action send_goal` prints nothing, and the log says `Failed to send goal response … (timeout)` | a start-up race between a brand-new command-line client and the action server; just send the goal again |
| rover4 hesitates or reverses near walls and doorways | normal for a car-like robot: it needs room to turn (radius ≈ 0.4 m) and makes multi-point turns |

## Useful diagnostic commands

```bash
ros2 topic echo /robot_mode                    # MANUAL / AUTO
ros2 topic echo /explore/status                # what the explorer is doing
ros2 lifecycle get /bt_navigator               # should be "active [3]"
ros2 topic hz /scan                            # are sensors publishing?
ros2 run tf2_ros tf2_echo map base_footprint   # where does the robot think it is?
ros2 run tf2_tools view_frames                 # full TF tree as a PDF
ros2 doctor --report                           # general ROS health check
gz topic -l                                    # Gazebo-side topics
```

When you ask for help (an issue, a forum), include: the command you ran, the **first** error in the
output (not the last), and the output of `ros2 doctor --report`.
