# esp32_motor_control ROS 包（简要说明）

该包位于工作空间 `jetson_catkin_ws` 下，用于通过 ROS 控制 ESP32 上的双轴电机（Yaw/Pitch），并支持自动控制任务和心形轨迹示例。

首先用
ls /dev/ttyUSB*
检测串口设备

然后打开三个终端分别运行
roscore

roslaunch esp32_motor_control esp32_bridge.launch

`rosrun esp32_motor_control esp32_bridge.py _time_margin:=0.0`


rosrun esp32_motor_control motor_control.py

或者rosrun esp32_motor_control test_motor_control.py

## 1. 主要功能

- 将 ROS 消息转换为 ESP32 串口协议：
  - `TPY:XXX TPP:XXX TVY:XXX TVP:XXX TE:XX TPID:XX TD:XX`
- 控制策略切换（通过 `/esp32/control_strategy`）：
  - **普通定控**（pid_auto!=0 或无值）：角度定控任务 / 速度定控任务；
  - **PID 定控**（pid_auto==0）：角度 PID 定控任务 / 速度 PID 定控任务；
  - 其余情况为普通模式，仅通过 `/esp32/motor_cmd` 下发手动命令。
- 测试脚本：
  1. 角度定控 + 速度定控；
  2. 角度 PID 定控 + 速度 PID 定控；
  3. 控制电机绘制心形轨迹（Pitch 限制在 ±90° 内）。

## 2. 工程结构

工作空间大致结构如下：

```text
jetson_catkin_ws/
├── src/
│   ├── CMakeLists.txt
│   └── esp32_motor_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── msg/
│       │   └── MotorCommand.msg
│       ├── launch/
│       │   └── esp32_bridge.launch   # 建议通过此文件启动桥接节点
│       ├── scripts/
│       │   ├── esp32_bridge.py       # 串口桥接节点
│       │   └── test_motor_control.py # 综合测试脚本
└── ...
```

## 2.1 系统结构与控制流程图示

下面是一个推荐的整体结构示意（你可以将实际的系统截图或绘图替换为真正图片）：

```mermaid
flowchart LR
  A[上层 ROS 节点\n(控制逻辑/GUI)] -->|发布 /esp32/motor_cmd| B[esp32_bridge.py\n(ROS-串口桥)]
  B -->|串口 TPY/TPP/TVY/TVP/TE/TD| C[ESP32 固件]
  C -->|电平 & 脉冲| D[Yaw 步进电机]
  C -->|电平 & 脉冲| E[Pitch 步进电机]
  C -->|串口状态字符串| B
  B -->|发布 /esp32/status| F[状态监控节点\n(rqt_console 等)]
```

如果你使用的是 Markdown 预览而非 mermaid，可以用一张 PNG 图片来替换上面的流程图，例如：

```markdown
![系统结构示意图](docs/images/system_architecture.png)
```

在这种情况下，只需将手绘或建模出的系统结构图保存为 `docs/images/system_architecture.png` 即可。

## 3. 消息与协议

### 3.1 `MotorCommand.msg` 字段说明

消息定义位于：`src/esp32_motor_control/msg/MotorCommand.msg`

```text
float32 tpy        # TPY: Yaw 目标角度(°)，-180~180，空值表示速度控制
float32 tpp        # TPP: Pitch 目标角度(°)，-90~90，空值表示速度控制
float32 tvy        # TVY: Yaw 目标速度(°/s)，-360~360
float32 tvp        # TVP: Pitch 目标速度(°/s)，-360~360
uint8  te          # TE: 使能位，高位Yaw, 低位Pitch，0=使能, 1=失能
uint8  tpid        # TPID: PID 串环控制开关，0 开启 PID，其他值或无值关闭
uint8  td          # TD: 保留位

uint8  angle_auto  # 1: 触发角度自动/角度PID任务；否则不触发
uint8  speed_auto  # 1: 触发速度自动/速度PID任务；否则不触发
uint8  pid_auto    # 0: PID 定控模式；非 0 或无值：普通定控模式
```

> 注意：angle_auto / speed_auto 只是 ROS 侧的“控制策略触发开关”，真正的运动逻辑由 `esp32_bridge.py` 中实现。

### 3.2 串口文本协议

桥接节点会将 `MotorCommand` 映射为如下字符串发送给 ESP32：

```text
TPY:XXX TPP:XXX TVY:XXX TVP:XXX TE:XX TPID:XX TD:XX\n
```

- `XXX` 为角度/速度的十进制数值，当前实现默认保留一位小数，例如 `-30.0`、`45.5`；
- `TE` 为两位十进制，低 2 bit 有效，高位为 Yaw 使能，低位为 Pitch 使能；
- `TD` 为两位十进制保留字段，当前固定为 0 或上层自定义；
- 每条命令以换行符 `\n` 结束。

## 4. 编译与环境

### 4.1 先决条件

- 已安装 ROS（例如 ROS Melodic/Noetic，取决于你的系统）；
- 已在系统中安装 Python 串口库 `pyserial`；
- 已确认 ESP32 通过 USB 串口连接到本机。

### 4.2 编译步骤

在工作空间根目录 `jetson_catkin_ws` 下执行：

```bash
cd /mnt/c/Users/le_lianxiang/Desktop/jetson_catkin_ws
catkin_make
source devel/setup.bash
```

> Windows 下建议在 WSL/ROS 环境中执行上述命令，并根据实际路径调整 `cd` 路径。

## 5. 桥接节点

### 5.1 启动桥接节点

首先用
ls /dev/ttyUSB*
检测串口设备

推荐使用已有的 launch 文件（如果你已配置）：

```bash
roslaunch esp32_motor_control esp32_bridge.launch
```

或手动启动：

```bash
rosrun esp32_motor_control esp32_bridge.py _port:=/dev/ttyUSB0 _baudrate:=115200
```

可选参数：

- `~port`：串口设备名，默认 `/dev/ttyUSB0`
- `~baudrate`：波特率，默认 `115200`
- `~timeout`：串口超时时间(s)，默认 `1.0`

### 5.2 话题接口

- 订阅：
  - `/esp32/control_strategy` (`esp32_motor_control/MotorCommand`)
    - 只看 `angle_auto`、`speed_auto` 字段，用于切换自动/普通模式；
  - `/esp32/motor_cmd` (`esp32_motor_control/MotorCommand`)
    - 在普通模式下，通过该话题直接控制 TPY/TPP/TVY/TVP/TE/TPID/TD；
- 发布：
  - `/esp32/status` (`std_msgs/String`)
    - 原样转发 ESP32 通过串口返回的字符串，可用于调试和监控。

### 5.3 心跳机制

桥接节点内部会每 10 秒通过串口发送一次 `"PING"` 字符串，用于和 ESP32 之间的心跳检测。

## 6. 控制策略与示例

### 6.1 普通手动控制（`/esp32/motor_cmd`）

当未处于自动任务模式时，桥接节点直接将收到的 `MotorCommand` 映射为协议指令发送给 ESP32。

简单示例：

```python
from esp32_motor_control.msg import MotorCommand
import rospy

rospy.init_node('manual_control_demo')
pub = rospy.Publisher('/esp32/motor_cmd', MotorCommand, queue_size=10)
rospy.sleep(1.0)

cmd = MotorCommand()
cmd.angle_auto = 0   # 注意：/esp32/motor_cmd 下发时这几个策略字段会被桥接节点忽略
cmd.speed_auto = 0
cmd.tpy = 30.0   # Yaw +30°
cmd.tpp = 0.0    # Pitch 0°
cmd.tvy = 60.0   # Yaw 60°/s
cmd.tvp = 60.0   # Pitch 60°/s
cmd.te = 0x00    # 两轴使能
cmd.tpid = 0     # 0 开启 PID 精确控制（由 ESP32 端实现）
cmd.td = 0

pub.publish(cmd)
```

### 6.2 策略话题 `/esp32/control_strategy` 概览

- 普通定控（`pid_auto != 0` 或不赋值）：
  - `angle_auto == 1`：角度定控任务；
  - `speed_auto == 1`：速度定控任务；
  - 两者都为 `0`：不触发任务，恢复普通模式（等待 `/esp32/motor_cmd`）；
- PID 定控（`pid_auto == 0`）：
  - `angle_auto == 1`：角度 PID 定控任务；
  - `speed_auto == 1`：速度 PID 定控任务；
  - 两者都为 `0`：不触发任务，恢复普通模式（等待 `/esp32/motor_cmd`）；
- 其他组合：回到普通模式，仅处理 `/esp32/motor_cmd`。

简单触发示例：

```python
# 角度定控任务（普通）
msg = MotorCommand()
msg.pid_auto = 1
msg.angle_auto = 1
msg.speed_auto = 1
strategy_pub.publish(msg)

# 角度 PID 定控任务
msg = MotorCommand()
msg.pid_auto = 0
msg.angle_auto = 1
msg.speed_auto = 1
strategy_pub.publish(msg)

```

## 7. 测试脚本 `test_motor_control.py`

文件位置：`src/esp32_motor_control/scripts/test_motor_control.py`

该脚本会按顺序自动执行三大阶段：

1. **角度定控 + 速度定控**（普通模式）：

- 使用 `/esp32/control_strategy` 触发角度定控任务，等待完成后暂停 5 秒；
- 再触发速度定控任务运行约 12 秒，再暂停 5 秒。

1. **角度 PID 定控 + 速度 PID 定控**：

- 使用 `/esp32/control_strategy` 触发角度 PID 定控任务，等待完成后暂停 5 秒；
- 再触发速度 PID 定控任务运行约 12 秒，再暂停 5 秒。

1. **心形轨迹绘制**：

- 使用心形曲线参数方程生成一系列 (Yaw, Pitch) 角度点，通过 `/esp32/motor_cmd` 下发；
- Pitch 角度始终裁剪在 [-90°, 90°] 范围内满足约束；
- 设置合适的角速度使轨迹连贯，持续约 10~15 秒。

### 7.2 心形轨迹示意图

脚本中使用的心形轨迹参数方程为：

$$
\begin{aligned}
x(t) &= 16\sin^3(t) \\
y(t) &= 13\cos(t) - 5\cos(2t) - 2\cos(3t) - \cos(4t),\\
&\quad t \in [0, 2\pi]
\end{aligned}
$$

在实现中，将 $x(t)$、$y(t)$ 缩放并分别映射到 Yaw (TPY)、Pitch (TPP) 角度：

- $\mathrm{TPY} = x(t) \times 3.0$（Yaw 角，大致在 [-50^\circ, 50^\circ]）；
- $\mathrm{TPP} = \mathrm{clip}(y(t) \times 2.0,\ -90^\circ,\ 90^\circ)$（Pitch 角，裁剪到 [-90^\circ, 90^\circ]）。

你可以将下图保存为 `docs/images/heart_trajectory.png`，并在需要时用 Markdown 预览：

```markdown
![心形轨迹示意图](docs/images/heart_trajectory.png)
```

图片建议使用 Matplotlib 或其它绘图工具，直接根据上述参数方程生成 $(x(t), y(t))$ 轨迹，能直观反映电机端期望的二维路径形状。

### 7.1 运行方式

在桥接节点已启动的前提下，在另一终端中执行：

```bash
cd /mnt/c/Users/le_lianxiang/Desktop/jetson_catkin_ws
source devel/setup.bash
rosrun esp32_motor_control test_motor_control.py
```

你可以在终端和 `/esp32/status` 话题中观察 ESP32 返回的状态信息，以确认命令执行情况。

## 8. 常见问题

1. **无法导入 `rospy` 或消息类型**
   - 确认已执行 `source devel/setup.bash`；
   - 确认 `catkin_make` 编译成功且无错误。

2. **串口无法打开**
   - 检查 `~port` 参数是否正确（如 `/dev/ttyUSB0`、`/dev/ttyACM0` 等）；
   - 在 Linux 下确认用户有访问串口权限（需要加入 `dialout` 组等）。

3. **ESP32 无响应**
   - 使用串口工具（如 `minicom`、`screen`）直接连接串口，确认 ESP32 固件已按协议解析命令；
   - 检查波特率、数据位、停止位配置是否一致。

4. **轨迹不平滑或越界**
   - 可以在 `test_motor_control.py` 中调整心形轨迹的采样点数、等待时间、速度或缩放系数；
   - Pitch 越界时会被裁剪到 [-90°, 90°]，可以根据实际机械结构适当减小缩放比例。

---

以上是本包的核心用法。更多调试或扩展（例如可视化、服务接口）可以根据自身工程需求自行添加。
