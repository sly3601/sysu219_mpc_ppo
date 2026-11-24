# Keyboard Input

This node will read the keyboard input and publish a control_input_msgs/Input message.

Tested environment:
* Ubuntu 24.04
  * ROS2 Jazzy
* Ubuntu 22.04
  * ROS2 Humble

### Build Command
```bash
cd ~/sysu219_mpc_ppo/ros2_ws
colcon build --packages-up-to keyboard_input
```

### Launch Command
```bash
source ~/sysu219_mpc_ppo/ros2_ws/install/setup.bash
ros2 run keyboard_input keyboard_input
```

## 1. Use Instructions for Sysu219 Guide

### 1.1 Control Mode
The keys map directly to the `command` field of `control_input_msgs/msg/Inputs`:

When launched with `policy:=hybrid`, keys `3`, `4`, `5`, `6`, `7`, and `9` select PPO
policies from settled FIXEDSTAND. Key `3` selects the latest feature-swing policy and key
`4` selects Apex. These keys are ignored while trotting. Press `2` and wait for FIXEDSTAND
before selecting another policy. The old assignments for these six keys have been removed.

| Key | command | Effect |
| --- | --- | --- |
| 1 | 1 | PASSIVE (kp=0, kd=1 passive damping), always available |
| 2 | 2 | Cycle the fixed poses: PRONE → DOWN → STAND → DOWN → STAND → ... |
| 3–7, 9 | 3–7, 9 | Select the corresponding Hybrid PPO policy from settled FIXEDSTAND |
| 8 | 8 | PRONE: from FIXED STAND it goes to DOWN first and then continues to PRONE automatically |
| 0 | 10 | Reserved |

* Starting from PASSIVE, the first press of `2` goes to PRONE, then DOWN, then STAND, and afterwards
  `2` toggles between DOWN and STAND.
* `1` switches to PASSIVE from any state, and is not blocked by the pose transition lock.
* Wait for each fixed-pose transition to finish (about 2 seconds) before pressing the next mode key.
* While trotting, `2` returns to FIXEDSTAND and `1` enters PASSIVE. Space clears the walking
  command. Ctrl+C closes the keyboard node; return to FIXEDSTAND or PASSIVE first.

### 1.2 Control Input
* WASD IJKL: Move robot
* Space: command back to 0 and reset speed input

## 2. FixedStand Offset Keyboard

The same package also installs `fixedstand_offset_keyboard`, a small debug node that publishes a
per-joint offset on `/fixedstand_offset_cmd`. It is not started by any launch file.

```bash
ros2 run keyboard_input fixedstand_offset_keyboard
```

* `[` / `]`: select the current joint (0~11)
* `-` / `+`: decrease / increase the current joint offset by 0.01 rad
* `r`: clear the current joint offset
