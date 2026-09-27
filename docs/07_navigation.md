# 07 · Navigation (Nav2)

Nav2 turns "go to (x, y)" into wheel commands. All parameters are in
`src/rover_navigation/config/nav2_params.yaml`, and the servers are started by
`rover_navigation/launch/navigation.launch.py`.

## The servers

| Server | Job | Plugin used here |
|---|---|---|
| `bt_navigator` | runs a **behavior tree** that coordinates everything below | default "navigate with replanning and recovery" tree |
| `planner_server` | global path on the whole map | **Smac Planner 2D** (A* on the costmap) |
| `controller_server` | follows the path at 20 Hz, reacting to nearby obstacles | **MPPI** (Model Predictive Path Integral) |
| `smoother_server` | smooths paths on request | simple smoother |
| `behavior_server` | recovery actions: spin, back up, wait | spin / backup / wait / drive_on_heading |
| `velocity_smoother` | limits acceleration so motion isn't jerky | – |
| `collision_monitor` | a last safety check on every command | footprint "approach" model |
| `lifecycle_manager` | starts the servers in the right order | – |

The **behavior tree** plans a path, then follows it, and replans every second. If following
fails, it clears the costmaps, spins or backs up, and tries again, a few times, before giving up.

## Global planner: Smac 2D

A* search over the global costmap. `allow_unknown: true` lets it plan through unexplored space,
which is essential for exploration. `cost_travel_multiplier: 2.0` makes cells near obstacles
"expensive", so paths stay centred in corridors and doorways.

> This project first used NavFn and switched to Smac 2D after a test run showed NavFn failing to
> plan when the robot was already a few centimetres from the goal. This is a good example of
> choosing a plugin based on evidence.

## Controller: MPPI

MPPI simulates **1000 random trajectories** (`batch_size`) over 2.5 s (`time_steps × model_dt`) at
every control step. It scores them with **critics** and blends the best ones:

| Critic | Rewards |
|---|---|
| `CostCritic` | staying away from costly cells (obstacles) |
| `PathAlignCritic` / `PathFollowCritic` / `PathAngleCritic` | following the global path |
| `GoalCritic` / `GoalAngleCritic` | reaching the goal position and heading |
| `PreferForwardCritic` | driving forwards (the camera faces forward!) |
| `ConstraintCritic` | respecting velocity limits |

Speed limits: `vx_max: 0.35`, `vx_min: -0.20`, `wz_max: 1.5`.

## Costmaps: where the sensors meet the planner

A costmap is a grid where each cell is FREE (0), INSCRIBED/LETHAL (the robot would collide), or
somewhere in between (inflated, "better not"). This robot uses **two** costmaps:

| | Local costmap | Global costmap |
|---|---|---|
| Frame / size | `odom`, 3 × 3 m rolling window | `map`, whole map |
| Used by | controller (MPPI), collision checks | planner |
| Layers | **VoxelLayer** (lidar + depth) → inflation | static (map) → obstacle (lidar) → **depth VoxelLayer** → inflation |

### Why the depth camera needs special care

The lidar is a single horizontal plane at 15.6 cm. A 10 cm brick is invisible to it. The depth
camera sees the brick, but:

* **Floor points must be ignored**, otherwise the whole floor is an obstacle. So
  `min_obstacle_height: 0.04` (4 cm).
* **Table tops above the robot must be ignored**, so `max_obstacle_height: 0.35`.
* **The lidar must not erase what the camera saw.** In a flat (2D) layer, a lidar ray passing
  *above* the brick clears the brick's cell. That's why the depth data goes into a separate
  **VoxelLayer** (3D): each ray only clears the heights it actually passes through.

This design was validated with a test goal whose straight path crossed a 10 cm brick: the robot
went around it with 0.41 m of clearance.

> **Known behaviour while mapping:** whenever SLAM enlarges the map, the global costmap is resized
> and forgets depth-only obstacles. The local costmap still sees them, and it is what does the
> avoiding. With a saved map (AMCL) this doesn't happen.

### Footprint and inflation

* `footprint`: a 22 × 22 cm square polygon (the real outline plus padding). A polygon is more
  accurate than a circle for a boxy robot.
* `inflation_radius: 0.35`, `cost_scaling_factor: 4.0`: cost decays over 35 cm from obstacles.
  Raising the scaling factor lets the robot get closer; lowering it keeps it further away.

## Collision monitor: the last line of defence

It sits between Nav2 and the mode manager. Every 0.1 s it simulates the robot's footprint along the
**current** command for `time_before_collision: 0.6` s. If lidar or depth points would be hit, it
slows the command down, down to a stop. It is independent of the planner, so it still protects
the robot if the planner makes a mistake.

`source_timeout: 2.0`: if a sensor goes quiet for longer than this, the monitor stops the robot
(fail-safe). It is set generously because a slow simulator can deliver camera frames late.

## Recoveries and the progress checker

`progress_checker`: if the robot moves less than 0.3 m in 15 s, it's "stuck", and the behavior tree
starts recoveries. `general_goal_checker`: goals count as reached within 0.20 m and 0.30 rad.

## Tuning cheat sheet

| Symptom | Try |
|---|---|
| Too slow / too fast | MPPI `vx_max`, `velocity_smoother.max_velocity` |
| Cuts corners, grazes walls | increase `inflation_radius` or `cost_scaling_factor` ↓; CostCritic `cost_weight` ↑ |
| Won't go through doorways | `inflation_radius` ↓ (≥ 0.16, half the robot's diagonal) |
| Stops and starts near obstacles | collision monitor `time_before_collision` ↓ |
| Wobbly path following | PathAlign/PathFollow `cost_weight` ↑, `wz_std` ↓ |
| Floor shows up as obstacles | depth `min_obstacle_height` ↑ |
| CPU overloaded | MPPI `batch_size` ↓, `controller_frequency` ↓ |

Many parameters can be changed live, for example:

```bash
ros2 param set /local_costmap/local_costmap inflation_layer.inflation_radius 0.25
```

## Use Nav2 from your own code

See `src/rover_control/rover_control/waypoint_patrol.py`, which uses the **Nav2 Simple Commander**:

```python
nav = BasicNavigator()
nav.waitUntilNav2Active(localizer='robot_localization')
nav.goToPose(pose)
while not nav.isTaskComplete():
    ...
result = nav.getResult()   # SUCCEEDED / CANCELED / FAILED
```

Run it with `ros2 run rover_control waypoint_patrol --ros-args -p use_sim_time:=true`.
