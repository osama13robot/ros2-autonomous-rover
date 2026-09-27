# 10 · Hands-on labs

Eight guided exercises, from "what is a topic?" to writing your own node. Each lab has a
**goal**, **steps**, ✅ **checkpoints** to confirm you're on track, and 🧠 **questions** to think
about. Suggested answers are at the end.

Before each lab, in every terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2-autonomous-rover/install/setup.bash
cd ~/ros2-autonomous-rover
```

> **Tip:** `headless:=true` makes everything faster; RViz shows all you need.

---

## Lab 1 · The robot model

*Time: ≈ 45 min*

**Goal:** understand the URDF, the TF frames, and the "single source of truth" design.

1. Show the robot: `ros2 launch rover_description display.launch.py`.
   Move the wheel sliders in the *Joint State Publisher* window.
2. In RViz, enable **TF → Show Names**. Find `base_footprint`, `base_link`, `lidar_link`,
   `camera_link` and `camera_optical_link`.
3. Print the generated URDF and find the lidar joint:
   ```bash
   xacro src/rover_description/urdf/rover.urdf.xacro sim_gazebo:=false | grep -A4 'joint name="lidar_joint"'
   ```
4. Open `src/rover_description/config/dimensions.yaml` and change:
   * `lidar.riser_height: 0.010` → `0.040` (lidar 3 cm higher)
   * `camera.pitch: 0.0` → `0.2` (camera tilted down about 11°)
5. Regenerate the parts and rebuild, then show the robot again:
   ```bash
   python3 hardware/generate_parts.py --check
   colcon build --symlink-install
   ros2 launch rover_description display.launch.py
   ```

✅ The lidar sits higher on a taller riser, and the camera looks down.
✅ `hardware/stl/lidar_riser.stl` is now 40 mm tall.

6. Revert both values, then regenerate and rebuild again.

🧠 **Q1.** Why is `base_footprint` on the ground but `base_link` on the wheel axle?
🧠 **Q2.** The collision geometry of the chassis is one box, but the visual is detailed meshes. Why?
🧠 **Q3.** Name one advantage and one drawback of tilting the camera down.

---

## Lab 2 · Exploring the ROS graph

*Time: ≈ 45 min*

**Goal:** learn the command-line tools every ROS developer uses daily.

Start the simulation: `./scripts/start_sim.sh headless:=true`.

1. Nodes and topics:
   ```bash
   ros2 node list
   ros2 topic list
   ros2 topic info /cmd_vel --verbose      # who publishes? who subscribes?
   ```
2. Data rates and content:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /camera/depth/points
   ros2 topic echo /robot_mode
   ros2 topic echo --once /scan --field range_max
   ```
3. The graph: `rqt_graph` (untick *Hide debug*, and choose *Nodes/Topics (all)*).
4. The TF tree: `ros2 run tf2_tools view_frames`, then open `frames_*.pdf`.
5. Where is the lidar, relative to the robot?
   ```bash
   ros2 run tf2_ros tf2_echo base_footprint lidar_link
   ```
6. Parameters:
   ```bash
   ros2 param list /mode_manager
   ros2 param get /mode_manager max_linear
   ```

✅ `/scan` runs at about 10 Hz (less if the sim is slower than real time).
✅ `tf2_echo` shows translation `[-0.010, 0.000, 0.156]`.
✅ `/cmd_vel` has exactly **one** publisher: `mode_manager`.

🧠 **Q4.** Why does `/robot_mode` show a value immediately, even though it's only published on changes?
🧠 **Q5.** Which node publishes `map → odom`? Which publishes `odom → base_footprint`?

---

## Lab 3 · Odometry and sensor fusion

*Time: ≈ 60 min*

**Goal:** see why the robot fuses wheel odometry and the IMU, and why SLAM is still needed.

1. Start the empty world: `./scripts/start_sim.sh headless:=true world:=empty.sdf`.
2. In a second terminal, watch the filtered estimate:
   ```bash
   ros2 topic echo /odometry/filtered --field pose.pose.position
   ```
3. In a third terminal, watch Gazebo's ground truth:
   ```bash
   gz topic -e -t /world/empty_world/dynamic_pose/info | grep -A3 'name: "rover"'
   ```
4. With the keyboard, drive a square of about 1 m sides (forward about 4 s at 0.25 m/s, then
   turn 90°, four times) and come back to the start.
5. Compare where the EKF thinks the robot is with the ground truth.
6. Look at the raw wheel odometry too: `ros2 topic echo /wheel/odometry_cov --field pose.pose.position`.

✅ In simulation the error is small: wheels don't slip much on a perfect floor.

7. **Experiment:** in `src/rover_navigation/config/ekf.yaml`, set the last `true` of
   `imu0_config` to `false` (so the IMU yaw rate is no longer used). Restart and repeat the square.
   Put it back afterwards.

🧠 **Q6.** Why does the EKF fuse *velocities* from the wheels instead of the position they report?
🧠 **Q7.** On a real robot on carpet, which would you trust more for turning: wheels or gyro? Why?
🧠 **Q8.** If the EKF is so good, why do we still need SLAM or AMCL?

---

## Lab 4 · Build a map with SLAM

*Time: ≈ 45 min*

**Goal:** map the world by hand, watch loop closure, save and reuse the map.

1. `./scripts/start_sim.sh` (keep the Gazebo GUI this time if your computer can handle it).
2. Stay in MANUAL. Drive slowly through **two rooms** and back to the start.
3. Watch the map in RViz while driving. Grey (unknown) becomes white (free) and black (walls).
4. When you come back near the start, look for a small "snap" of the map: that's a **loop closure**.
5. Save it: `./scripts/save_map.sh lab4`.
6. Open `src/rover_navigation/maps/lab4.pgm` in any image viewer, and read `lab4.yaml`.
7. Restart on your map (with AMCL instead of SLAM):
   ```bash
   ./scripts/start_sim.sh slam:=false map:=$PWD/src/rover_navigation/maps/lab4.yaml
   ```

✅ The robot appears on your map at the right place. Driving, the lidar dots stay aligned with the walls.

🧠 **Q9.** Driving fast and turning fast make SLAM maps worse. Why?
🧠 **Q10.** What does the `origin` field in `lab4.yaml` mean?

---

## Lab 5 · Sending navigation goals

*Time: ≈ 45 min*

**Goal:** command Nav2 in three different ways.

Start on the included map: `./scripts/start_sim.sh headless:=true slam:=false map:=$PWD/src/rover_navigation/maps/rover_world.yaml`,
then press `2` in the keyboard window (AUTO).

1. **RViz:** use *2D Goal Pose* to click a goal in another room. Watch the blue global path and the robot.
2. **Command line (action):**
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 5.0, y: 3.0}, orientation: {w: 1.0}}}}" --feedback
   ```
3. **Python:** the patrol example.
   ```bash
   ros2 run rover_control waypoint_patrol --ros-args -p use_sim_time:=true \
     -p waypoints:="[1.5, 2.0, 5.0, 3.0, 0.0, 0.0]"
   ```
4. While the patrol runs, press `w` in the keyboard window. What happens to the patrol?
5. Read `src/rover_control/rover_control/waypoint_patrol.py`. It's short.

✅ The action prints feedback (distance remaining) and ends with `SUCCEEDED`.
✅ Pressing a drive key cancels the patrol ("Goal cancelled (manual take-over?)").

🧠 **Q11.** What's the difference between a topic, a service and an action? Why is navigation an action?
🧠 **Q12.** Why must the robot be in AUTO for the goal to move it?

---

## Lab 6 · Costmaps and the depth camera

*Time: ≈ 60 min*

**Goal:** see how the depth camera protects the robot from obstacles the lidar can't see.

1. Start on the included map in AUTO (as in Lab 5).
2. Send a goal whose straight path crosses the **low red brick** at (2.0, −1.2):
   ```bash
   ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     "{pose: {header: {frame_id: map}, pose: {position: {x: 3.6, y: -1.85}, orientation: {w: 1.0}}}}"
   ```
3. In RViz, watch the **local costmap** as the brick comes into the camera's view. It appears as an
   obstacle even though there are no red lidar dots on it.

✅ The robot curves around the brick.

4. **Experiment (the robot will hit the brick):** in `src/rover_navigation/config/nav2_params.yaml`,
   change `observation_sources: scan depth` to `observation_sources: scan` in the **local costmap**,
   remove `depth_layer` from the global costmap `plugins` list, and set `enabled: False` for the
   collision monitor's `depth` source. Restart and send the same goal (drive back to the origin first).
5. **Restore the file** (for example `git checkout src/rover_navigation/config/nav2_params.yaml`).

🧠 **Q13.** Why is the depth data kept in a *voxel* layer instead of a normal 2D obstacle layer?
🧠 **Q14.** Why is `min_obstacle_height` 0.04 rather than 0.0?

---

## Lab 7 · Tune the explorer

*Time: ≈ 60–90 min*

**Goal:** understand how parameter choices change autonomous behaviour.

1. Run a baseline exploration and time it:
   ```bash
   ./scripts/start_sim.sh headless:=true initial_mode:=auto
   ros2 topic echo /explore/status        # note the sim time when it says DONE
   ros2 topic echo --once /clock          # current sim time
   ```
2. Edit `src/rover_bringup/config/control.yaml` → `frontier_explorer`, one change per run:
   * **Greedy nearest:** `size_weight: 0.0`
   * **Big frontiers first:** `distance_weight: 0.2`
   * **Brave:** `clearance: 0.20`
3. Record, for each run: time to `DONE`, number of "Abandoning" or "Could not reach" messages in the
   launch output, and how complete the map looks.

✅ You have a small table comparing three strategies.

🧠 **Q15.** Why might "greedy nearest" produce a lot of back-and-forth driving?
🧠 **Q16.** What would go wrong if `done_confirmations` were 1?

---

## Lab 8 · Write your own node

*Time: ≈ 60 min*

**Goal:** create a ROS 2 Python package from scratch: a *proximity guard* that reports the closest
obstacle and warns when something is too near.

1. Create a package inside the workspace:
   ```bash
   cd ~/ros2-autonomous-rover/src
   ros2 pkg create --build-type ament_python my_rover_tools \
     --dependencies rclpy sensor_msgs std_msgs --node-name proximity_guard
   ```
2. Replace `my_rover_tools/my_rover_tools/proximity_guard.py` with:
   ```python
   import math

   import rclpy
   from rclpy.node import Node
   from rclpy.qos import qos_profile_sensor_data
   from sensor_msgs.msg import LaserScan
   from std_msgs.msg import Float32


   class ProximityGuard(Node):
       def __init__(self):
           super().__init__('proximity_guard')
           self.declare_parameter('warn_distance', 0.35)
           self.pub = self.create_publisher(Float32, '/closest_obstacle', 10)
           self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)

       def on_scan(self, scan):
           valid = [(r, i) for i, r in enumerate(scan.ranges)
                    if math.isfinite(r) and scan.range_min < r < scan.range_max]
           if not valid:
               return
           dist, idx = min(valid)
           angle = math.degrees(scan.angle_min + idx * scan.angle_increment)
           self.pub.publish(Float32(data=dist))
           if dist < self.get_parameter('warn_distance').value:
               self.get_logger().warn(f'Obstacle {dist:.2f} m away at {angle:+.0f} deg',
                                      throttle_duration_sec=1.0)


   def main():
       rclpy.init()
       node = ProximityGuard()
       try:
           rclpy.spin(node)
       except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
           pass                        # Ctrl-C: exit quietly
       finally:
           node.destroy_node()
           rclpy.try_shutdown()


   if __name__ == '__main__':
       main()
   ```
3. Build and run it:
   ```bash
   cd ~/ros2-autonomous-rover
   colcon build --symlink-install --packages-select my_rover_tools
   source install/setup.bash
   ros2 run my_rover_tools proximity_guard --ros-args -p use_sim_time:=true
   ```
4. With the sim running, drive towards a wall and watch the warnings. Also run
   `ros2 topic echo /closest_obstacle`.

✅ Warnings appear below 0.35 m, and the angle tells you where the obstacle is (0° = front, +90° = left).

**Challenges:**
* Publish a `visualization_msgs/Marker` arrow pointing at the closest obstacle and show it in RViz.
* Ignore obstacles behind the robot (only consider −60°…+60°).
* Make it change the robot's speed limit: study how `mode_manager` clamps speeds, and add a
  `/speed_scale` topic it listens to.

🧠 **Q17.** Why subscribe with `qos_profile_sensor_data` rather than the default QoS?

---

## Suggested answers

<details><summary>Click to reveal</summary>

**Q1.** REP-105: planners and costmaps work in 2D on the floor, so a ground-level frame is convenient
(`base_footprint`). Mechanical parts are naturally described from the axle (`base_link`).

**Q2.** Physics engines are much faster and more stable with simple shapes; meshes are only for looks.

**Q3.** Advantage: it sees low obstacles close to the robot. Drawback: it sees less far ahead and
less of tall obstacles, and more floor (so floor filtering matters more).

**Q4.** It's *latched* (`TRANSIENT_LOCAL` durability): the publisher re-sends the last message to
new subscribers.

**Q5.** `map → odom`: slam_toolbox (or AMCL). `odom → base_footprint`: the EKF (`ekf_filter_node`).

**Q6.** Positions from wheel odometry already contain accumulated error. Fusing velocities lets
the EKF integrate them itself and combine them properly with the other sensors.

**Q7.** The gyro. Wheels slip and skid on carpet, especially when turning; a gyro measures rotation
directly.

**Q8.** Any odometry (even fused) is integrated, so small errors pile up without limit. Only
comparing with the environment (lidar vs. map) corrects the drift.

**Q9.** Scan matching assumes consecutive scans overlap well and that the odometry guess is close.
Fast motion breaks both, and motion during a single lidar rotation distorts the scan.

**Q10.** The position of the lower-left pixel of the image in the `map` frame (x, y, yaw).

**Q11.** Topic: continuous one-way stream. Service: quick request/response. Action: a long task
with feedback and cancellation. Navigating takes time, reports progress, and must be cancellable,
so it's an action.

**Q12.** The mode manager only forwards Nav2's commands in AUTO. In MANUAL, Nav2 computes commands
but they never reach the wheels.

**Q13.** In a 2D layer, lidar rays passing *above* a low obstacle would clear its cell. In 3D, each
ray only clears the voxels it passes through.

**Q14.** Noise and small errors in the camera's pose would make floor points appear slightly above
zero, and the whole floor would become an obstacle.

**Q15.** It always takes the closest frontier, even a tiny one, and may leave big areas half-explored
and have to come back later.

**Q16.** Right after start-up the map is nearly empty, so there may be no valid frontier on the
first check. The explorer would end immediately (this actually happened during development).

**Q17.** `qos_profile_sensor_data` is *best effort* with a small queue: for sensors, a fresh
message matters more than a resent old one. A best-effort subscriber can receive from both reliable
and best-effort publishers, but a default *reliable* subscriber gets nothing from a best-effort
publisher, which many real lidar and camera drivers use.

</details>
