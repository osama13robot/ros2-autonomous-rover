#!/usr/bin/env bash
# One-click simulation: Gazebo + RViz + autonomy stack, plus a keyboard-teleop window.
#   ./start_sim.sh                      (extra args are passed to the launch file)
#   ./start_sim.sh initial_mode:=auto   (start exploring right away)
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if pgrep -f "gz sim" >/dev/null; then
  echo "WARNING: a Gazebo server is already running (left over from a previous run?)."
  echo "Two simulators publish on the same topics and confuse the robot. Stop it with:"
  echo "    pkill -f 'gz sim'"
  read -r -p "Continue anyway? [y/N] " answer
  [[ "$answer" == [yY]* ]] || exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

if command -v gnome-terminal >/dev/null; then
  gnome-terminal --title="Rover keyboard" -- bash -c \
    "source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'; sleep 8; ros2 run rover_control teleop_keyboard; exec bash" || true
else
  echo "Open another terminal and run:  ros2 run rover_control teleop_keyboard"
fi

exec ros2 launch rover_bringup sim.launch.py "$@"
