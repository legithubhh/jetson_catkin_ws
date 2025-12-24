# Copilot instructions (esp32_motor_control)

## Big picture (ROS ↔ ESP32)
- This repo is a ROS catkin workspace. The main package is `src/esp32_motor_control`.
- The ROS↔ESP32 boundary is the node `scripts/esp32_bridge.py`: it converts ROS `MotorCommand` messages into a serial text protocol and schedules “auto tasks”.
- Data flow:
  - Publish manual control to `/esp32/motor_cmd` (`esp32_motor_control/MotorCommand`) → `esp32_bridge.py` → serial command line.
  - Publish strategy to `/esp32/control_strategy` (`MotorCommand`) → `esp32_bridge.py` starts/stops built-in tasks.
  - ESP32 serial feedback is forwarded as raw text on `/esp32/status` (`std_msgs/String`). Lines starting with `MPU_STATUS:` are logged as WARN.

## Serial protocol (source of truth)
- Protocol string is built in `scripts/esp32_bridge.py` as:
  - `TPY:{tpy} TPP:{tpp} TVY:{tvy} TVP:{tvp} TE:{te:02d} TPID:{tpid:02d} TD:{td:02d}\n`
- `te` is masked to 2 bits: `te = int(msg.te) & 0x03`.
- Convention used by the bridge:
  - `TE=3` means **pause/stop now** (stops auto tasks and sends a stop command).
  - `TPID=0` enables PID mode on ESP32; `TPID=0xFF` is used as “PID off/ignore”.

## Strategy switching (be careful: doc vs msg comments)
- Implementation is in `ESP32Bridge.control_strategy_callback`.
- **Trigger values in code:** `angle_auto==1` / `speed_auto==1` start tasks.
- Mode selection:
  - `pid_auto == 0` → PID tasks (`start_angle_pid_task`, `speed_pid_task`).
  - otherwise → “normal” tasks (`start_angle_normal_task`, `speed_normal_task`).
- Priority:
  - Angle task wins when both `angle_auto` and `speed_auto` are `1`.
  - Manual `/esp32/motor_cmd` has higher priority: receiving a manual cmd requests auto-stop first.

## Built-in tasks (where to change motion)
- Angle normal task: `start_angle_normal_task` (relative deltas +36/-36, +90/-90; uses `time_margin`).
- Angle PID task: `start_angle_pid_task` (absolute targets -60/-120; pitch -90/0; `TPID=0`).
- Speed normal task: `speed_normal_task` uses constants (`yaw_speed`, `pitch_speed`, `dt`) to generate synchronized periodic motion.
- Speed PID task: `speed_pid_task` cycles through fixed angle points.

## Attitude logging + keyboard commands
- The bridge keeps an **estimated** attitude (not sensor truth) and appends to `~attitude_file` (default `motor_attitude.log`).
- A background stdin loop accepts:
  - `R`: return-to-origin (shortest path) using the last attitude record
  - `K`: clear log and set current position as new origin

## Developer workflow (catkin)
- Build messages/nodes after changes to `msg/MotorCommand.msg` or `CMakeLists.txt`:
  - `catkin_make` then `source devel/setup.bash`
- Run-time (typical):
  - `roscore`
  - `roslaunch esp32_motor_control esp32_bridge.launch` (sets `~port`, `~baudrate`, `~timeout`, `~time_margin`)
  - Optional test drivers: `rosrun esp32_motor_control motor_control.py` or `rosrun esp32_motor_control test_motor_control.py`

## Repo-specific gotchas
- `scripts/test_import.py` appears outdated (it references non-existent `MotorCommand` fields like `speed/steps/direction/enable`). Avoid using it as an API reference.
- Scripts use `#!/usr/bin/env python` and include Python2-compatible input handling; avoid Python3-only syntax if keeping ROS Melodic/Kinetic compatibility matters.
