#!/usr/bin/env bash
# One-click simulation: Gazebo + RViz + autonomy stack, plus a keyboard-teleop window.
#   ./start_sim.sh                      (extra args are passed to the launch file)
#   ./start_sim.sh initial_mode:=auto   (start exploring right away)
#   ./start_sim.sh robot:=rover4        (the 4WD Ackermann robot)
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

# the car-like rover4 cannot turn in place: start the keyboard in car mode
TELEOP_ARGS=""
for arg in "$@"; do
  [[ "$arg" == "robot:=rover4" ]] && TELEOP_ARGS="--ros-args -p ackermann:=true"
done

if command -v gnome-terminal >/dev/null; then
  gnome-terminal --title="Rover keyboard" -- bash -c \
    "source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'; sleep 8; ros2 run rover_control teleop_keyboard $TELEOP_ARGS; exec bash" || true
else
  echo "Open another terminal and run:  ros2 run rover_control teleop_keyboard $TELEOP_ARGS"
fi

exec ros2 launch rover_bringup sim.launch.py "$@"
