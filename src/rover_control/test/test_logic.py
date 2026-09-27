"""Unit tests for the pure logic of the rover_control nodes (no ROS graph needed)."""
import numpy as np
import rclpy
from nav_msgs.msg import MapMetaData

from rover_control.frontier_explorer import FrontierExplorer
from rover_control.mode_manager import ModeManager
from rover_control.teleop_keyboard import TeleopKeyboard


def make_info(res=0.05, w=100, h=100):
    info = MapMetaData()
    info.resolution = res
    info.width, info.height = w, h
    return info


def test_frontiers_found_between_free_and_unknown():
    rclpy.init()
    try:
        node = FrontierExplorer()
        grid = -np.ones((100, 100), dtype=np.int8)
        grid[:, :50] = 0            # left half known free, right half unknown
        frontiers, cells = node.find_frontiers(grid, make_info())
        assert len(frontiers) == 1
        x, y, size = frontiers[0]
        assert abs(x - 49.5 * 0.05) < 0.06     # on the boundary column
        assert size > 4.0                      # ~5 m long frontier
        assert node.still_frontier(grid, make_info(), (x, y))
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_frontiers_skip_walls_and_tiny_gaps():
    rclpy.init()
    try:
        node = FrontierExplorer()
        grid = np.zeros((100, 100), dtype=np.int8)
        grid[:, 60] = 100           # wall
        grid[:, 61:] = -1           # unknown behind the wall
        grid[50, 60] = 0            # 1-cell gap: too small to count
        frontiers, _ = node.find_frontiers(grid, make_info())
        assert frontiers == []
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_clamp_rejects_nan_and_limits():
    assert ModeManager.clamp(float('nan'), 1.0) == 0.0
    assert ModeManager.clamp(5.0, 0.5) == 0.5
    assert ModeManager.clamp(-5.0, 0.5) == -0.5


def test_teleop_ramp():
    assert TeleopKeyboard.ramp(0.0, 0.3, 0.05) == 0.05
    assert TeleopKeyboard.ramp(0.28, 0.3, 0.05) == 0.3
    assert TeleopKeyboard.ramp(0.3, 0.0, 0.05) == 0.0     # stop at once


def test_teleop_ackermann_turn_keys_roll_forward():
    rclpy.init()
    try:
        node = TeleopKeyboard()
        node.ackermann = True
        node.mode = 'MANUAL'
        node.handle_key('a')
        assert node.target == (1, 1)          # steer left while driving forwards
        node.handle_key('d')
        assert node.target == (1, -1)
        node.ackermann = False
        node.handle_key('a')
        assert node.target == (0, 1)          # diff drive: rotate in place
        node.destroy_node()
    finally:
        rclpy.shutdown()
