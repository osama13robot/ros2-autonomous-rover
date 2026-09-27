#!/usr/bin/env bash
# Save the current SLAM map:  ./save_map.sh [name]   -> src/rover_navigation/maps/<name>.yaml/.pgm
# Re-use it later with:  ros2 launch rover_bringup sim.launch.py slam:=false map:=<path to yaml>
set -e
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="${1:-my_map}"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
ros2 run nav2_map_server map_saver_cli -f "$WS/src/rover_navigation/maps/$NAME" \
  --ros-args -p use_sim_time:=true -p save_map_timeout:=20.0
echo "Saved $WS/src/rover_navigation/maps/$NAME.yaml"
