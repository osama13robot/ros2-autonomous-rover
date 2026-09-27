# 04 · Robot design

![robot](images/robot_render.png)

## Concept

A classic **two-deck differential-drive** robot, the same layout as TurtleBot-style platforms:

* two driven wheels on a common axle at the centre, so it can **turn in place**
* front and rear ball casters for balance
* a lower deck for motors, battery and computer, and an upper deck for the lidar (clear 360° view)
* an RGB-D camera low at the front, where it sees what the lidar misses

| Property | Value |
|---|---|
| Size (L × W × H) | 200 × 206 × 171 mm |
| Mass | ~1 kg |
| Wheel radius / separation | 35 mm / 182 mm |
| Top speed (sim limit) | 0.5 m/s, autonomy uses ≤ 0.35 m/s |
| Lidar scan height | 156 mm |
| Camera height | 77 mm, 88 mm ahead of the axle |

## One source of truth: `dimensions.yaml`

`src/rover_description/config/dimensions.yaml` holds every dimension, mass and sensor setting.
Two very different programs read it:

```
dimensions.yaml ──► rover.urdf.xacro ──► simulation, TF frames, Nav2
               └──► hardware/generate_parts.py ──► printable STL files (+ meshes shown in the sim)
```

So if you make the wheels bigger, both the printed parts and the simulated robot change together.
This is the most important design idea in the project: **the model can't drift away from the real robot**.

In xacro, the YAML is loaded with:

```xml
<xacro:property name="d" value="${xacro.load_yaml('$(find rover_description)/config/dimensions.yaml')}"/>
<xacro:property name="R" value="${d['wheel']['radius']}"/>
```

## URDF structure

`urdf/rover.urdf.xacro` is organised as:

| Block | Contents |
|---|---|
| properties | derived heights (all relative to the wheel axle), wheel positions |
| `base_link` | both decks (STL meshes), standoffs, motors, electronics; **one** simple box for collision |
| `wheel` macro | a continuous joint about Y; visual = printed hub + tire meshes, collision = cylinder |
| `caster` macro | fixed sphere with **zero friction**, so it slides like a free ball |
| sensors | `imu_link`, `lidar_link`, `camera_link` + `camera_optical_link` |
| `rover.gazebo.xacro` | simulation-only plugins and sensors (included when `sim_gazebo:=true`) |

Design choices worth noticing:

* **Visual and collision geometry are different.** Visuals use detailed meshes (they look good);
  collisions use boxes, cylinders and spheres, which are fast and stable for the physics engine.
* **Inertia is computed by macros** (`inertial_macros.xacro`) from mass and size. Wrong inertia is a
  classic cause of robots that shake or fly away in simulation.
* **`camera_optical_link`** is rotated so that Z points forward, X right and Y down. This is the
  convention for camera images ([REP-103](https://www.ros.org/reps/rep-0103.html)); robot bodies use
  X forward, Y left, Z up.

## Sensors

| Sensor | Real part | Simulated as | Why it's there |
|---|---|---|---|
| 2D lidar | RPLIDAR C1 | `gpu_lidar`, 360 samples, 0.12–12 m, 10 Hz, σ = 1 cm | SLAM and localization need long-range 360° geometry |
| RGB-D camera | RealSense D435 | `rgbd_camera`, 424 × 240, 87° FOV, 0.2–6 m, 15 Hz | the lidar is a single plane at 15.6 cm, so the camera catches **lower and higher** obstacles |
| IMU | BNO055 / ICM-20948 | `imu`, 100 Hz, realistic noise and bias | its gyro measures rotation directly, even when the wheels slip |
| Wheel encoders | JGA25-370 hall encoders | `DiffDrive` odometry + `JointStatePublisher` | measure distance travelled; the backbone of odometry |

Each sensor covers a weakness of another:

* **Wheel odometry** is accurate over short distances but drifts, and it is wrong when a wheel slips.
* The **IMU gyro** is right during slips but drifts in heading over time.
* **Lidar SLAM** corrects long-term drift, but it only sees one horizontal plane.
* The **depth camera** sees in 3D, but only forward and at short range.

Combining them (sensor fusion) gives a system more reliable than any single sensor.

## Viewing and editing the robot

```bash
ros2 launch rover_description display.launch.py                  # with STL meshes
ros2 launch rover_description display.launch.py use_meshes:=false  # simple shapes
xacro src/rover_description/urdf/rover.urdf.xacro > /tmp/rover.urdf && check_urdf /tmp/rover.urdf
```

After changing `dimensions.yaml`:

```bash
python3 hardware/generate_parts.py --check   # new STL parts + sim meshes
colcon build --symlink-install               # meshes are installed files
```

Lab 1 in [Hands-on labs](10_hands_on_labs.md) walks you through this.

## Printable hardware

See the [Hardware guide](../hardware/README.md) for the parts list, BOM, print settings, assembly and wiring.

![parts](images/parts_preview.png)

The parts generator is itself a small lesson in computational geometry. Every part is a set of
**2.5D extrusions** (a 2D outline with holes, pushed up to a thickness). The script triangulates each
outline with **ear clipping** (holes are joined to the outline with "bridge" edges), builds the walls
and writes binary STL. `--check` verifies that every edge is shared by exactly two triangles, which
means the part is watertight and printable.
