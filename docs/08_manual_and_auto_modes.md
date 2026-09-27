# 08 · Manual and autonomous modes

## The design: one gate to the wheels

```
teleop_keyboard ──/cmd_vel_teleop──┐
                                   ├──► mode_manager ──/cmd_vel──► robot
Nav2 ──────────────/cmd_vel_nav────┘         ▲
                                  /mode_request, /set_autonomous
```

`mode_manager` (`src/rover_control/rover_control/mode_manager.py`) is a **velocity
multiplexer**, the only node allowed to publish `/cmd_vel`:

| Mode | Forwards | Ignores |
|---|---|---|
| `MANUAL` | `/cmd_vel_teleop` | `/cmd_vel_nav` |
| `AUTO` | `/cmd_vel_nav` | `/cmd_vel_teleop` |

Why this design? Without a single gate, the keyboard and Nav2 would fight over the wheels, 20 times
a second each. With a gate:

* only one source ever reaches the robot,
* switching modes is a single state change,
* safety rules live in exactly one place.

## Safety behaviours

| Behaviour | How |
|---|---|
| **Dead-man** | the gate publishes a steady 20 Hz stream. If the selected source has been silent for `teleop_timeout` / `nav_timeout` (0.5 s), the stream is **zero**. A crashed node or a lost message can never leave the robot driving. |
| **No stale commands** | on every mode switch the last stored commands are discarded and a zero is sent. |
| **Clean hand-over** | switching AUTO → MANUAL **cancels every Nav2 goal**, so the robot never resumes an old goal by surprise. |
| **Hard speed clamp** | `max_linear` / `max_angular` apply in both modes; NaN or infinite values become 0. |
| **Take-over** | in the keyboard node, any drive key pressed in AUTO requests MANUAL first. |

How can the gate cancel goals it didn't send? Every ROS 2 action server offers a
`<action>/_action/cancel_goal` service. A request with an all-zero goal id and time stamp means
"cancel all goals". The mode manager calls this for `navigate_to_pose`,
`navigate_through_poses` and `follow_waypoints`.

## Interfaces

| Name | Kind | Use |
|---|---|---|
| `/robot_mode` | topic `std_msgs/String`, **latched** | current mode; late subscribers still get it |
| `/mode_request` | topic `std_msgs/String` | `"manual"`, `"auto"` or `"toggle"` |
| `/set_autonomous` | service `std_srvs/SetBool` | `true` = AUTO |

A "latched" topic uses the QoS `durability: TRANSIENT_LOCAL`: the publisher keeps the last message
and hands it to anyone who subscribes later. Subscribers must use the same QoS.

## The keyboard node

`teleop_keyboard.py` puts the terminal into *cbreak* mode (keys arrive immediately, without Enter)
and polls it with `select()` 20 times a second.

A terminal cannot report key **releases**. "Hold to drive" is therefore emulated: while a key is
held, the terminal repeats it about 30 times a second, and motion continues until no repeat has
arrived for `key_timeout` (0.6 s, longer than the typical 0.5 s repeat delay). For classic latched
behaviour (press once, keep going) use:

```bash
ros2 run rover_control teleop_keyboard --ros-args -p hold_to_drive:=false
```

Other parameters: `linear_speed`, `angular_speed`, `accel_linear`, `accel_angular` (a smooth ramp
up; stopping is immediate), `take_over_auto`.

## Parameters (`rover_bringup/config/control.yaml`)

```yaml
mode_manager:
  ros__parameters:
    initial_mode: manual
    teleop_timeout: 0.5
    nav_timeout: 0.5
    max_linear: 0.5
    max_angular: 2.5
    cancel_nav_on_manual: true
```

## Ideas to extend it

* Add a third mode, `ASSISTED`: keyboard commands filtered by the collision monitor. Nav2's
  `assisted_teleop` behaviour already exists.
* Add a gamepad: `teleop_twist_joy` remapped to `/cmd_vel_teleop`, with a button that publishes to `/mode_request`.
* Add an `ESTOP` mode that also latches until explicitly reset.
