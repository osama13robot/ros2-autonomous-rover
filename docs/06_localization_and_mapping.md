# 06 · Localization and mapping

"Where am I?" is answered in **two layers**, matching the two upper links of the TF tree:

```
map ──(SLAM or AMCL: corrects drift, may jump)──► odom ──(EKF: smooth, drifts)──► base_footprint
```

## Layer 1: the EKF (`robot_localization`)

**Goal:** a smooth, continuous estimate of motion at 30 Hz.

**Inputs** (`rover_navigation/config/ekf.yaml`):

| Source | Fused values | Why only these |
|---|---|---|
| `/wheel/odometry_cov` | forward speed `vx`, sideways speed `vy` (always 0), yaw rate `vyaw` | fuse **velocities**, not positions: they don't accumulate error inside the sensor |
| `/imu/data` | yaw rate `vyaw` | a gyro measures rotation directly, even when the wheels slip |

The 15-element `*_config` arrays select which of
`[x y z, roll pitch yaw, vx vy vz, vroll vpitch vyaw, ax ay az]` to use from each sensor.

`two_d_mode: true` tells the filter that the robot lives on a flat floor.

**Output:** the TF `odom → base_footprint` and the topic `/odometry/filtered`.

> **Why not use the wheel odometry directly?** Try [Lab 3](10_hands_on_labs.md#lab-3--odometry-and-sensor-fusion).
> When a wheel slips, wheel-only odometry believes the robot turned when it didn't. The gyro knows better.

## Layer 2a: SLAM Toolbox (mapping mode, the default)

**Goal:** build a map and keep the robot located in it. SLAM Toolbox is a graph-based SLAM:

1. Each new lidar scan (after the robot moves ≥ 0.2 m or turns ≥ 0.2 rad) is **scan-matched**
   against recent scans, which refines the odometry guess.
2. Scans become **nodes in a pose graph**, and the matches become edges.
3. When the robot returns to a known place, a **loop closure** adds an edge between distant nodes.
   The graph is then optimised (with the Ceres solver) and accumulated drift is removed.
4. The occupancy grid `/map` is re-rendered from the corrected poses every `map_update_interval`
   (2 s here).

It publishes `map → odom` (the correction) and `/map`.

Key parameters in `slam_toolbox.yaml`:

| Parameter | Value | Effect |
|---|---|---|
| `resolution` | 0.05 | 5 cm grid cells |
| `max_laser_range` | 12.0 | uses the full lidar range |
| `minimum_travel_distance` / `heading` | 0.2 / 0.2 | how often scans are added |
| `do_loop_closing` | true | the drift-removal magic |
| `map_update_interval` | 2.0 | faster map updates help the explorer |

**A detail you may notice:** right after start-up the map is almost empty. Cells only count as free
after a lidar ray has passed through them more than once, and new scans are only added after the
robot moves. The explorer handles this by requiring several empty evaluations in a row before it
declares a map "complete".

### Saving a map

```bash
./scripts/save_map.sh my_map
```

This runs `nav2_map_server map_saver_cli` and writes `my_map.pgm` (the image) and `my_map.yaml`
(resolution, origin, thresholds). In the image, black is occupied, white is free and grey is unknown.

## Layer 2b: AMCL (localization on a saved map)

With `slam:=false map:=<yaml>`, `map_server` publishes the stored map and **AMCL** (Adaptive Monte
Carlo Localization) tracks the robot on it:

1. It keeps 500–2000 **particles**, each one a guess of the robot pose.
2. When the robot moves, every particle moves by the odometry change plus noise (`alpha1..5`).
3. Each particle is scored by how well the current lidar scan fits the map from its pose
   (the likelihood field model).
4. Particles are resampled in proportion to their scores; wrong guesses die out.

It publishes `map → odom`. The start pose is set in `nav2_params.yaml`
(`set_initial_pose: true`, `initial_pose` at the origin). If you spawn the robot elsewhere, give AMCL
a hint with **2D Pose Estimate** in RViz.

## SLAM or AMCL?

| | SLAM (`slam:=true`) | AMCL (`slam:=false`) |
|---|---|---|
| Needs a map beforehand | no | yes |
| Map changes over time | yes (it keeps mapping) | no (fixed) |
| Exploration | ✅ | ✖ (nothing left to explore) |
| CPU | higher | lower |
| Typical use | first visit, changing places | daily operation in a known place |

## Measured accuracy (simulation)

* End of a full autonomous exploration: SLAM pose vs. Gazebo ground truth ≈ **1 cm**.
* AMCL on the saved map after crossing into another room: ≈ **6 cm**.
