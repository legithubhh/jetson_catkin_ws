# ROS1 Noetic → ROS2 Humble 迁移指导（esp32_motor_control）

本仓库保留 ROS1(catkin) 版本，并新增 ROS2(colcon) 版本在 `ros2_ws/`。

## 一、目录结构（方案 A：拆包）
- `ros2_ws/src/esp32_motor_control_msgs`
  - 仅包含接口：`msg/MotorCommand.msg`
- `ros2_ws/src/esp32_motor_control`
  - rclpy 节点：`esp32_bridge`（串口桥接 + 自动任务调度）
  - 测试脚本：`motor_control`、`test_motor_control`
  - ROS2 launch：`launch/esp32_bridge.launch.py`

## 二、关键行为与兼容性
- 策略触发（以实现为准）：
  - `angle_auto == 1` / `speed_auto == 1` 触发任务
  - `pid_auto == 0` 选择 PID 任务，否则 normal 任务
- 暂停/急停：向 `/esp32/control_strategy` 发送 `TE=3`
- 串口协议：由桥接节点拼接文本并以 `\n` 分包。
- 速度定控任务1：ROS2 版本使用 ESP32 的“速度控制模式”（TPY/TPP 置空）以获得更匀速的连续输出。

## 三、构建（colcon）
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 四、运行
### 1) 启动桥接节点（推荐 launch）
```bash
ros2 launch esp32_motor_control esp32_bridge.launch.py
```

### 2) 手动指定串口参数
```bash
ros2 run esp32_motor_control esp32_bridge --ros-args \
  -p port:=/dev/ttyUSB0 -p baudrate:=115200 -p timeout:=1.0 -p time_margin:=0.1
```

### 3) 启动测试驱动
```bash
# 直接启动速度定控任务1，退出即 TE=3 暂停
ros2 run esp32_motor_control motor_control

# 综合测试（默认只跑 normal 部分；可按需取消注释 PID/心形）
ros2 run esp32_motor_control test_motor_control
```

## 五、验证
```bash
ros2 topic list
ros2 topic echo /esp32/status
```

如果看到串口回传的 `MPU_STATUS:` 行，桥接节点会以 WARN 级别打印，便于 rqt_console/日志筛查。
