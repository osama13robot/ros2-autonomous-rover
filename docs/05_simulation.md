# 05 · Simulation

## The stack

```
Gazebo Harmonic (gz-sim 8)  ◄── ros_gz_sim "create" spawns the URDF from /robot_description
        │  gz-transport topics (/scan, /camera/points, /cmd_vel …)
        ▼
ros_gz_bridge parameter_bridge  (config: rover_gazebo/config/gz_bridge.yaml)
        │  ROS 2 topics
        ▼
rover_control/sim_sensor_adapter  →  the rest of the ROS system
```

Gazebo has its **own** messaging system (gz-transport), separate from ROS 2. The **bridge**
translates the topics listed in `gz_bridge.yaml`, in the direction given there:

```yaml
- ros_topic_name: /scan
  gz_topic_name: /scan
  ros_type_name: sensor_msgs/msg/LaserScan
  gz_type_name: gz.msgs.LaserScan
  direction: GZ_TO_ROS
```

Try `gz topic -l` (Gazebo side) next to `ros2 topic list` (ROS side) to see both worlds.

## The world: `rover_world.sdf`

A 14 × 10 m apartment/workshop built from simple static shapes:

| Feature | Why it's there |
|---|---|
| rooms connected by 1.2–1.5 m doorways | exploration has to find its way through doors |
| pillars, crates, a barrel | plenty of lidar features for SLAM |
| tables with thin legs | the lidar sees only four small dots, a realistic challenge |
| **low bricks, a step, a bin (8–10 cm)** | **below the lidar plane**, so only the depth camera can detect them |

`empty.sdf` is a flat ground, useful for testing the drive base.

The world file also loads the **system plugins** Gazebo needs:

```xml
<plugin filename="gz-sim-physics-system" .../>        <!-- physics -->
<plugin filename="gz-sim-sensors-system" ...>          <!-- rendering sensors: lidar, camera -->
  <render_engine>ogre2</render_engine>
</plugin>
<plugin filename="gz-sim-imu-system" .../>            <!-- IMU sensors -->
<plugin filename="gz-sim-scene-broadcaster-system" .../> <!-- GUI -->
<plugin filename="gz-sim-user-commands-system" .../>  <!-- spawning -->
```

A sensor declared in the URDF produces nothing unless its world-level system is loaded. This is a
very common beginner problem.

## Robot plugins (`rover.gazebo.xacro`)

* **DiffDrive** turns `/cmd_vel` into wheel speeds, with acceleration limits, and publishes
  wheel odometry. Its own TF output is deliberately *not* bridged, because the EKF publishes
  `odom → base_footprint`.
* **JointStatePublisher** publishes wheel angles on `/joint_states`, so the wheels turn in RViz.
* **Sensors** use `<gz_frame_id>` so their messages carry the correct URDF frame names.

## The sim sensor adapter: making simulation look like real drivers

`rover_control/sim_sensor_adapter.py` fixes three differences between Gazebo and real hardware
drivers:

1. **Covariances.** Gazebo's odometry and IMU messages don't carry meaningful covariances (how
   uncertain each value is). The EKF would then over-trust them. The adapter fills in realistic
   uncertainties, so the EKF weighs its sources properly.
2. **The point-cloud frame quirk.** Gazebo's RGB-D sensor publishes its point cloud with
   *x-forward* (body) axes while stamping it with the *optical* frame (z-forward). Left as is, every
   obstacle would appear in the wrong place. The adapter re-stamps the cloud with `camera_link`.
   This was found by measuring: the cloud's median point had x ≈ 0.8 m (forward) and z ≈ −0.07 m
   (the floor).
3. **Downsampling.** It keeps every 2nd pixel in each direction (4× fewer points), so the costmaps
   stay fast.

On the real robot this node is not used; the drivers publish these topics directly.

## Time in simulation

Every node runs with `use_sim_time: true` and reads the clock from `/clock` (Gazebo's time). If
the simulator runs at 0.5× real time, the whole robot "thinks" at half speed, and everything stays
consistent. Watch the real-time factor in the bottom-right corner of the Gazebo GUI.

## Performance tips

| Change | Effect |
|---|---|
| `headless:=true` | biggest win: no 3D GUI rendering |
| lower `camera.rate` or `image_width/height` in `dimensions.yaml` | less rendering work |
| lower MPPI `batch_size` in `nav2_params.yaml` | less CPU for the controller |
| `world:=empty.sdf` | fewer objects to render |

## Make your own world

1. Copy `rover_world.sdf` to `src/rover_gazebo/worlds/my_world.sdf`, keeping the `<plugin>` and
   `<light>` blocks.
2. Add models: a static `<model>` with a `<link>`, a `<collision>` and a `<visual>` (see the `box`
   and `cylinder` examples in the file), or include models from
   [Gazebo Fuel](https://app.gazebosim.org/fuel) with `<include><uri>https://fuel.gazebosim.org/...</uri></include>`.
3. Validate it: `gz sdf -k src/rover_gazebo/worlds/my_world.sdf` (prints `Valid.`).
4. Rebuild (a new file must be installed), then run `./scripts/start_sim.sh world:=my_world.sdf`.
