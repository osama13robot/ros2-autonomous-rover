# 01 · Installation

Target platform: **Ubuntu 24.04 (Noble) + ROS 2 Jazzy + Gazebo Harmonic**. These three are an
official matching set; other combinations (for example Humble + Gazebo Classic) will not work
without changes.

> **Not on Ubuntu 24.04?** Use a virtual machine or a dual boot. WSL2 on Windows 11 can run it
> (with WSLg for the GUI), but 3D rendering is slow; use `headless:=true` there.

## 1. Install ROS 2 Jazzy

Follow the official guide: <https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html>.
It comes down to the following steps (check the guide for the latest version):

```bash
# locale + repositories
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
# the ROS apt source (the official guide installs the ros2-apt-source package)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools
```

Check that it works:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker     # Ctrl-C to stop
```

## 2. Install the project's dependencies

```bash
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  python3-scipy
```

`ros-jazzy-ros-gz` pulls in **Gazebo Harmonic**. Check it with `gz sim --versions` (expect 8.x).

## 3. Get and build the workspace

```bash
git clone <this-repo-url> ~/ros2-autonomous-rover
cd ~/ros2-autonomous-rover
source /opt/ros/jazzy/setup.bash
rosdep update && rosdep install --from-paths src --ignore-src -y   # catches anything missing
colcon build --symlink-install
```

`--symlink-install` links Python files and configs instead of copying them. You can then edit a
`.yaml`, `.py` or launch file and simply restart the program, with no rebuild. You only need to
rebuild after adding new files or changing `setup.py` / `CMakeLists.txt`.

## 4. Source the workspace

Every new terminal needs:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2-autonomous-rover/install/setup.bash
```

To make this automatic, add both lines to the end of `~/.bashrc`.

## 5. Verify

```bash
colcon test --packages-select rover_control && colcon test-result --verbose
# → Summary: 4 tests, 0 errors, 0 failures, 0 skipped

ros2 launch rover_description display.launch.py
# → RViz shows the robot; move the wheel sliders
```

You're ready for the [Quick start](02_quick_start.md).

## Optional: the 3D-printable parts

The STL files are already in `hardware/stl/`. To regenerate them after changing dimensions you only
need Python 3 with numpy and PyYAML, which ROS 2 already installs:

```bash
python3 hardware/generate_parts.py --check
```
