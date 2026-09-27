# 13 · Future possibilities

The rover is a platform. Here are directions to take it, from weekend projects to research topics.
Each one lists what you'd learn and where to start in the code.

Difficulty: 🟢 beginner · 🟡 intermediate · 🔴 advanced

## Perception

| Project | | What you'd build | Starting point |
|---|---|---|---|
| **Object detection** | 🟡 | run a YOLO model on `/camera/color/image_raw`, publish `vision_msgs/Detection2DArray` | a new node; the camera is already bridged |
| **3D object positions** | 🟡 | combine detections with the depth image, so the robot knows *where* a chair is | depth is on `/camera/depth/image_raw` |
| **Semantic map** | 🔴 | label map regions ("kitchen", "door") from detections | a new costmap layer, or markers |
| **Person following** | 🟡 | detect a person, send moving goals, or drive directly in AUTO | `waypoint_patrol.py` is the template |
| **ArUco / AprilTag docking** | 🟡 | find a tag on a charging dock and drive onto it precisely | Nav2's `opennav_docking` is installed with Nav2 |
| **Cliff detection** | 🟢 | treat a *missing* floor in the depth image as an obstacle (stairs!) | extend the depth pipeline in `sim_sensor_adapter` |

## Mapping and localization

| Project | | What you'd build | Starting point |
|---|---|---|---|
| **3D RGB-D SLAM** | 🔴 | RTAB-Map with the depth camera for 3D maps and visual loop closure | `ros-jazzy-rtabmap-ros`, replace `slam.launch.py` |
| **Lifelong mapping** | 🟡 | keep updating a saved map as furniture moves | SLAM Toolbox `lifelong` mode |
| **Multi-floor maps** | 🔴 | switch maps when changing floor | Nav2 map server `load_map` service |
| **Better exploration** | 🟡 | information-gain frontiers, planner-based distances | `frontier_explorer.py`, see guide 09 |

## Navigation and behaviour

| Project | | What you'd build | Starting point |
|---|---|---|---|
| **Patrol scheduler** | 🟢 | patrol routes at set times, with a report of what was seen | `waypoint_patrol.py` |
| **Custom behavior tree** | 🟡 | a Nav2 BT XML, for example "try the goal; if blocked, wait 10 s, then try another route" | `bt_navigator` `default_nav_to_pose_bt_xml` |
| **Keepout / speed zones** | 🟢 | forbid areas and slow down near people | Nav2 costmap filters (a mask image) |
| **Assisted teleop mode** | 🟢 | keyboard driving that refuses to crash | Nav2 `assisted_teleop`, `mode_manager.py` |
| **Gamepad control** | 🟢 | `teleop_twist_joy` → `/cmd_vel_teleop`, a button → `/mode_request` | guide 08 |
| **Return to charge** | 🟡 | a battery model: when low, go to the dock | a battery plugin in Gazebo + docking |

## Interfaces

| Project | | What you'd build | Starting point |
|---|---|---|---|
| **Web dashboard** | 🟡 | map, camera, mode buttons in a browser (phone-friendly) | `rosbridge_suite` + roslibjs, or Foxglove |
| **Voice / natural-language commands** | 🟡 | "go to the kitchen", with a language model turning text into goals | named places → poses → `goToPose` |
| **Telemetry and logging** | 🟢 | record runs with `ros2 bag record`, analyse them offline | `ros2 bag` |

## Engineering

| Project | | What you'd build | Starting point |
|---|---|---|---|
| **Docker dev container** | 🟢 | one command to get a working environment on any OS | an `osrf/ros:jazzy-desktop-full` base image |
| **Simulation tests in CI** | 🟡 | a headless test that explores the world and checks the map area | the CI workflow in `.github/workflows` |
| **ros2_control** | 🟡 | replace the Gazebo DiffDrive plugin with `diff_drive_controller`, the same controller used on real hardware | `gz_ros2_control` is available |
| **C++ ports** | 🟡 | rewrite `mode_manager` in C++ (composable node) for lower latency | a good C++ ROS 2 exercise |
| **Parameter sweeps** | 🟡 | automatically run exploration with many parameter sets and compare | Lab 7, automated |

## Hardware

| Project | | What you'd build |
|---|---|---|
| **Bumper ring** | 🟢 | printed bumper + microswitches → a `/bumper` topic → an emergency stop |
| **Battery monitor** | 🟢 | an INA219 current/voltage sensor → `sensor_msgs/BatteryState` |
| **Outdoor variant** | 🔴 | bigger wheels, GPS + `navsat_transform`, outdoor Nav2 |
| **Small arm** | 🔴 | a 3-DOF printed arm on the top deck, with MoveIt 2 |
| **Mecanum wheels** | 🟡 | a holonomic drive: MPPI `motion_model: Omni` |
| **Per-wheel speeds for rover4** | 🟡 | drive each of the 4 wheels at its own Ackermann speed in simulation too (ros2_control `ackermann_steering_controller` or a custom Gazebo system), removing tire scrub |
| **4-wheel steering (rover4 → 4WS)** | 🔴 | steer the rear axle as well: tighter turns, crab motion; needs a custom controller and planner model |

## Research directions

* **Reinforcement learning** for local navigation, trained in this Gazebo world (a gym wrapper
  around `/cmd_vel` and `/scan`).
* **Multi-robot exploration**: several rovers, namespaced, sharing a map and splitting frontiers.
* **Sim-to-real studies**: measure how parameters tuned in simulation transfer to the real robot,
  and add noise and latency models to close the gap.
* **Learning-based perception for costmaps**: learn which depth points are traversable
  (thick carpet vs. a real step).

## Contributing a project

Pick one, open an issue saying you're working on it, and see [CONTRIBUTING.md](../CONTRIBUTING.md).
Add a guide in `docs/` for anything you build. Future learners will thank you.
