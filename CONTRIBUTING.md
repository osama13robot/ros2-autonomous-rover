# Contributing

Thanks for helping! Anything that makes the project easier to learn from is welcome: fixing a
typo, clarifying a guide, adding a world, porting it to real hardware, or building one of the
[future possibilities](docs/13_future_possibilities.md).

## Workflow

1. Open an issue describing the bug or the idea (for bigger changes, before you start).
2. Fork the repository and create a branch: `git checkout -b feature/my-change`.
3. Make the change, then check it:
   ```bash
   colcon build --symlink-install
   colcon test --packages-select rover_control && colcon test-result --verbose
   python3 hardware/generate_parts.py --check        # if you touched dimensions or parts
   ./scripts/start_sim.sh headless:=true             # run it and watch it work
   ```
4. Open a pull request that explains **what** changed, **why**, and **how you tested it**.

## Guidelines

* **Keep it teachable.** Prefer clear code over clever code, and comment the *why*.
* **One source of truth.** Physical dimensions go in `dimensions.yaml`, never hard-coded elsewhere.
* **Document behaviour changes** in the matching guide in `docs/`.
* **Report test results honestly** in the PR, including what you did *not* test (for example
  "simulation only, not on hardware").
* Python follows PEP 8 (max line length 110), and ROS naming follows the
  [ROS 2 conventions](https://docs.ros.org/en/jazzy/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).

## Adding a world

Put it in `src/rover_gazebo/worlds/`, keep the system plugins from `rover_world.sdf`, validate it
with `gz sdf -k <file>`, and mention it in [05 Simulation](docs/05_simulation.md).
