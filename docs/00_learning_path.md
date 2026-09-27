# 00 · Learning path

This project is designed so you can learn from it in layers. First you run it and see it work,
then you understand how it works, then you change it.

## Who this is for

| You are… | Start with | Then |
|---|---|---|
| **New to ROS 2** (you know some Python and Linux) | [01 Installation](01_installation.md) → [02 Quick start](02_quick_start.md) | Labs 1–3, then guides 03 and 04 |
| **Familiar with ROS 2 basics** (nodes, topics, launch) | [02 Quick start](02_quick_start.md) → [03 Architecture](03_architecture.md) | Guides 06–09, then labs 4–7 |
| **Experienced with ROS / Nav2** | [03 Architecture](03_architecture.md) | [13 Future possibilities](13_future_possibilities.md): pick a project |
| **A maker who wants to build it** | [Hardware guide](../hardware/README.md) | [11 Real robot](11_real_robot.md) |
| **A teacher** | This page + [10 Hands-on labs](10_hands_on_labs.md) | Each lab is one class session (45–90 min) |

## Prerequisites

* **Computer:** Ubuntu 24.04, 4+ CPU cores, 8 GB RAM (16 GB recommended). A GPU helps a lot but is
  not required: the whole project was developed and tested with CPU-only rendering.
* **Skills:** basic terminal use (`cd`, `ls`, editing files) and reading Python. No robotics
  background is needed; concepts are explained as they come up.

## Suggested 4-week course

| Week | Topic | Guides | Labs |
|---|---|---|---|
| 1 | Run it, drive it, explore the ROS graph | 01, 02, 03 | 1, 2, 3 |
| 2 | Robot description and simulation | 04, 05 | (edit the robot in Lab 1) |
| 3 | Localization, mapping, navigation | 06, 07 | 4, 5, 6 |
| 4 | Autonomy and your own code | 08, 09 | 7, 8 + a project from guide 13 |

## How the pieces fit together (the 60-second version)

1. **Sensors** (lidar, depth camera, IMU, wheel encoders) publish data on **topics**.
2. An **EKF** fuses wheel odometry and IMU into a smooth estimate of how the robot moved.
3. **SLAM** uses the lidar to build a **map** and to work out where the robot is on it.
4. **Nav2** plans a path on that map and drives the robot along it while avoiding obstacles.
5. The **frontier explorer** picks the next unexplored spot and gives it to Nav2 as a goal.
6. The **mode manager** decides who is allowed to drive: you (keyboard) or Nav2.

## Glossary

| Term | Meaning |
|---|---|
| **Node** | a running program in ROS 2 (for example `mode_manager`) |
| **Topic** | a named data stream nodes publish to and subscribe from (for example `/scan`) |
| **Service** | a request/response call (for example `/set_autonomous`) |
| **Action** | a long-running goal with feedback and cancel (for example `navigate_to_pose`) |
| **TF** | the tree of coordinate frames (`map → odom → base_footprint → lidar_link …`) |
| **URDF / xacro** | XML description of the robot's links, joints, sensors; xacro adds variables and macros |
| **Odometry** | an estimate of motion from wheels/IMU; it drifts slowly over time |
| **EKF** | Extended Kalman Filter: combines noisy sensors into a better estimate |
| **SLAM** | Simultaneous Localization And Mapping: building a map while locating yourself in it |
| **AMCL** | Adaptive Monte Carlo Localization: locating the robot on an *existing* map using particles |
| **Costmap** | a grid saying how "expensive" (dangerous) each cell is for the robot |
| **Frontier** | the border between explored free space and unknown space |
| **Lifecycle node** | a node with managed states (unconfigured → inactive → active); Nav2 uses these |
| **Sim time** | in simulation, nodes use Gazebo's clock (`/clock`) instead of the wall clock |
