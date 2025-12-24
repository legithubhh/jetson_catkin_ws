#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""ESP32 串口桥接节点

本节点完成三类功能：
1. 将 ROS 话题 `/esp32/motor_cmd` 中的 `MotorCommand` 消息
    按照 ESP32 端约定的字符串协议
    `"TPY:XXX TPP:XXX TVY:XXX TVP:XXX TE:XX TPID:XX TD:XX"` 打包并通过串口发送。
2. 通过专用话题 `/esp32/control_strategy` 切换控制策略：
    - 普通角度/速度定控任务（pid_auto!=0 或无值）；
    - PID 角度/速度定控任务（pid_auto==0）；
    - 或保持普通模式，仅转发 `/esp32/motor_cmd`。
3. 周期性向 ESP32 发送 `PING` 心跳，并回传串口状态到 `/esp32/status`。

注意：
这里只实现协议适配和控制任务调度，不对 ESP32 反馈做复杂解析，
反馈原样发布到 `/esp32/status` 供上层使用。
"""

import math
import threading
import time

import rospy
import serial
from std_msgs.msg import String

from esp32_motor_control.msg import MotorCommand


class ESP32Bridge(object):
    """ROS 与 ESP32 之间的桥接类。"""

    def __init__(self):
        # ----------------- 串口及 ROS 基本参数 -----------------
        # 串口参数可以通过 rosparam 动态配置
        self.port = rospy.get_param("~port", "/dev/ttyUSB0")
        self.baudrate = rospy.get_param("~baudrate", 115200)
        self.timeout = rospy.get_param("~timeout", 1.0)

        # 自动任务的时间裕度（秒）：在理论运行时间的基础上再多等待的时间
        # 可通过命令行 / roslaunch 传入，例如：
        #   rosrun esp32_motor_control esp32_bridge.py _time_margin:=0.0
        self.time_margin = float(rospy.get_param("~time_margin", 0.5))
        if self.time_margin < 0.0:
            rospy.logwarn("~time_margin < 0, clamp to 0.0")
            self.time_margin = 0.0

        # 当前控制模式标志
        # True: 执行“速度类”自动任务（速度定控或速度PID定控）；False: 不执行
        self.speed_auto_mode = False
        # 角度类自动任务执行锁，防止并发执行（角度定控或角度PID定控）
        self.angle_task_lock = threading.Lock()
        # 自动任务中断标志：收到“暂停 / 手动优先”命令时置位
        self.stop_auto_flag = False

        # 电机姿态估计（Yaw / Pitch），以“当前设定的零点”为原点，单位：度，范围 [-180, 180]
        # 注意：这里是**基于指令和时间的估算值**，并非 MPU 真实反馈
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0
        # 姿态数据文本路径，可通过参数 ~attitude_file 自定义
        self.attitude_file = rospy.get_param("~attitude_file", "motor_attitude.log")
        self.attitude_lock = threading.Lock()

        # 控制策略：由 /esp32/control_strategy 话题驱动
        # 仅保存最近一次策略命令，真正的任务逻辑在回调中触发
        self.strategy_sub = rospy.Subscriber(
            "/esp32/control_strategy", MotorCommand, self.control_strategy_callback, queue_size=10
        )

        # 串口对象
        self.ser = None
        self.connect_serial()

        # 状态发布者：直接发布 ESP32 返回的原始字符串
        self.status_pub = rospy.Publisher("/esp32/status", String, queue_size=10)

        # 普通电机控制指令订阅者：仅在“正常模式”下使用
        self.cmd_sub = rospy.Subscriber(
            "/esp32/motor_cmd", MotorCommand, self.motor_cmd_callback, queue_size=10
        )

        # 串口读取线程，负责读取 ESP32 的反馈
        self.read_thread = threading.Thread(target=self.read_serial)
        self.read_thread.daemon = True
        self.read_thread.start()

        # 键盘指令线程：读取 R / K 命令
        # R: 从姿态文件读取当前姿态，让电机以最短路径回到“原点”；
        # K: 清空姿态文件，并将当前位置设为新的“原点”。
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()

        rospy.loginfo("ESP32 Bridge initialized on port %s", self.port)

    # ------------------------------------------------------------------
    # 姿态估计相关工具函数
    # ------------------------------------------------------------------
    @staticmethod
    def _wrap_angle_180(angle):
        """将任意角度归一化到 [-180, 180] 区间。

        这里只做简单的 360° 周期折返，不考虑极端数值情况。
        """

        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _append_attitude_to_file(self):
        """将当前姿态 (yaw, pitch) 追加写入姿态数据文本。

        每行格式：`timestamp yaw pitch`，单位分别为秒与度。
        """

        try:
            with open(self.attitude_file, "a") as f:
                f.write("{:.3f} {:.3f} {:.3f}\n".format(time.time(), self.yaw_angle, self.pitch_angle))
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("Failed to write attitude file %s: %s", self.attitude_file, exc)

    def _set_attitude(self, yaw_deg, pitch_deg):
        """将内部姿态估计直接设置为给定的绝对角度值（度）。

        假设电机已经到达该目标姿态，用于角度/速度 PID 任务。
        """

        with self.attitude_lock:
            self.yaw_angle = self._wrap_angle_180(float(yaw_deg))
            self.pitch_angle = self._wrap_angle_180(float(pitch_deg))
            self._append_attitude_to_file()

    def _add_attitude(self, delta_yaw_deg, delta_pitch_deg):
        """在当前姿态估计的基础上增加一个增量角度，并写入文件。

        适用于“速度定控任务1”与“角度定控任务1”等相对转动场景。
        """

        with self.attitude_lock:
            self.yaw_angle = self._wrap_angle_180(self.yaw_angle + float(delta_yaw_deg))
            self.pitch_angle = self._wrap_angle_180(self.pitch_angle + float(delta_pitch_deg))
            self._append_attitude_to_file()

    # ------------------------------------------------------------------
    # 串口相关
    # ------------------------------------------------------------------
    def connect_serial(self):
        """建立与 ESP32 的串口连接。"""

        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout,
            )
            rospy.loginfo("Successfully connected to ESP32 on %s", self.port)

            # 等待 ESP32 完成复位和启动
            time.sleep(2.0)
            self.read_initial_message()
        except serial.SerialException as exc:
            rospy.logerr("Serial connection error: %s", exc)
            rospy.signal_shutdown("Serial connection failed")

    def read_initial_message(self):
        """读取 ESP32 上电后的第一条启动信息（如果有）。"""

        try:
            if self.ser and self.ser.in_waiting > 0:
                initial_msg = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if initial_msg:
                    rospy.loginfo("ESP32: %s", initial_msg)
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("Failed to read initial message: %s", exc)

    # ------------------------------------------------------------------
    # ROS 订阅回调与控制策略调度
    # ------------------------------------------------------------------
    def control_strategy_callback(self, msg):
        """处理 `/esp32/control_strategy` 话题，用于切换控制策略。

        规则（来自最新 ROS 端控制计划）：

        当 pid_auto 为 1（或非 0，普通定控）：
        - angle_auto == 1: 切换为“角度定控任务1”；
        - 否则若 speed_auto == 1: 切换为“速度定控任务1”；
        - 否则（两个都为 0）：恢复普通模式，等待 `/esp32/motor_cmd`。

        当 pid_auto 为 0（PID 定控）：
        - angle_auto == 1: 切换为“角度 PID 定控任务1”；
        - 否则若 speed_auto == 1: 切换为“速度 PID 定控任务1”；
        - 否则（两个都为 0）：恢复普通模式，等待 `/esp32/motor_cmd`。

        特别地：当 angle_auto 与 speed_auto 都为 1 时，因为 angle_auto
        的判断优先，因此按照文档“也为角度控制任务”的约定，仍视为角度任务。
        """

        # 特殊：当 te == 3 时，认为是“暂停 / 停机”命令
        # 立即停止所有自动任务，并向 ESP32 发送停机指令
        if msg.te == 3:
            rospy.logwarn("[ESP32Bridge] Received PAUSE command from strategy, stop auto tasks & motors.")
            # 设置中断标志，让自动任务线程尽快退出
            self.stop_auto_flag = True
            # 退出速度类自动循环
            self.speed_auto_mode = False
            # 构造一个全 0 的停机命令，立即发送
            stop_msg = MotorCommand()
            stop_msg.tpy = 0.0
            stop_msg.tpp = 0.0
            stop_msg.tvy = 0.0
            stop_msg.tvp = 0.0
            stop_msg.te = 3
            stop_msg.tpid = 0xFF
            stop_msg.td = 0
            stop_cmd = self.build_protocol_command(stop_msg)
            rospy.loginfo("[ESP32Bridge] Send PAUSE cmd: %s", stop_cmd)
            self.send_command(stop_cmd)
            return

        # ---------- 根据 pid_auto 与 angle_auto / speed_auto 切换任务 ----------
        pid_mode_is_pid = (msg.pid_auto == 0)

        if not pid_mode_is_pid:
            # 普通定控模式（pid_auto != 0）
            if msg.angle_auto == 1:
                # 角度定控任务1
                self.stop_auto_flag = False
                self.speed_auto_mode = False
                rospy.loginfo("[ESP32Bridge] Strategy: angle NORMAL task1.")
                self.start_angle_normal_task()
                return

            if msg.speed_auto == 1:
                # 速度定控任务1
                self.stop_auto_flag = False
                if not self.speed_auto_mode:
                    self.speed_auto_mode = True
                    rospy.loginfo("[ESP32Bridge] Strategy: speed NORMAL task1.")
                    thread = threading.Thread(target=self.speed_normal_task)
                    thread.daemon = True
                    thread.start()
                return
        else:
            # PID 定控模式（pid_auto == 0）
            if msg.angle_auto == 1:
                # 角度 PID 定控任务1
                self.stop_auto_flag = False
                self.speed_auto_mode = False
                rospy.loginfo("[ESP32Bridge] Strategy: angle PID task1.")
                self.start_angle_pid_task()
                return

            if msg.speed_auto == 1:
                # 速度 PID 定控任务1
                self.stop_auto_flag = False
                if not self.speed_auto_mode:
                    self.speed_auto_mode = True
                    rospy.loginfo("[ESP32Bridge] Strategy: speed PID task1.")
                    thread = threading.Thread(target=self.speed_pid_task)
                    thread.daemon = True
                    thread.start()
                return

        # 未匹配到任何自动任务：恢复普通模式
        rospy.loginfo("[ESP32Bridge] Strategy: normal mode (wait /esp32/motor_cmd).")
        self.stop_auto_flag = False
        self.speed_auto_mode = False

    def motor_cmd_callback(self, msg):
        """处理来自 `/esp32/motor_cmd` 的电机控制命令（正常模式）。

        当未处于自动任务模式时，将 `MotorCommand` 直接映射为协议字符串并发送。
        """

        # 设计调整：手动命令优先级高于自动任务
        # 收到手动命令时，先请求停止自动任务，再直接下发该命令
        if self.speed_auto_mode or self.angle_task_lock.locked():
            rospy.logwarn("[ESP32Bridge] Auto task running, but MANUAL CMD has higher priority -> stop auto.")
            # 请求所有自动任务尽快退出
            self.stop_auto_flag = True
            self.speed_auto_mode = False

        cmd_str = self.build_protocol_command(msg)
        rospy.loginfo("[ESP32Bridge] Send manual cmd: %s", cmd_str)
        self.send_command(cmd_str)

    # ------------------------------------------------------------------
    # 协议封装
    # ------------------------------------------------------------------
    @staticmethod
    def _format_angle(value):
        """将角度/速度数值格式化为协议中的 `XXX` 字段。

        协议文本中没有给出精确的小数位约束，
        这里采用保留一位小数的字符串格式，例如 `-30.0`、`45.5`。
        如果值为 NaN/inf/None，则返回空字符串表示“无值”（用于进入 ESP32 端的速度控制模式）。
        """

        try:
            # 若值本身表示“无效”，返回空
            if value is None:
                return ""
            v = float(value)
            if math.isnan(v) or math.isinf(v):
                return ""
            return "{:.1f}".format(v)
        except (TypeError, ValueError):
            return ""

    def build_protocol_command(self, msg):
        """将 `MotorCommand` 消息映射为串口协议字符串。

        目标协议格式：
        `"TPY:XXX TPP:XXX TVY:XXX TVP:XXX TE:XX TD:XX"`

        字段含义见《ESP32端控制命令协议指导.md》。
        """

        tpy = self._format_angle(msg.tpy)
        tpp = self._format_angle(msg.tpp)
        tvy = self._format_angle(msg.tvy)
        tvp = self._format_angle(msg.tvp)

        # 使能位、PID 开关和保留字段直接以两位无符号整数发送
        te = int(msg.te) & 0x03  # 只保留低 2 bit
        tpid = int(msg.tpid) & 0xFF
        td = int(msg.td) & 0xFF

        # 拼接协议字符串，字段之间以空格分隔
        cmd = (
            "TPY:{tpy} TPP:{tpp} TVY:{tvy} TVP:{tvp} "
            "TE:{te:02d} TPID:{tpid:02d} TD:{td:02d}"
        ).format(
            tpy=tpy,
            tpp=tpp,
            tvy=tvy,
            tvp=tvp,
            te=te,
            tpid=tpid,
            td=td,
        )
        return cmd

    def send_command(self, command):
        """向 ESP32 发送一条完整命令字符串。

        :param command: 已经按照协议格式化好的字符串，不包含行结束符。"""

        try:
            if self.ser and self.ser.is_open:
                # 协议采用换行符分包
                self.ser.write((command + "\n").encode("utf-8"))
                self.ser.flush()
            else:
                rospy.logwarn("Serial port not open, attempting reconnect...")
                self.connect_serial()
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logerr("Error sending command: %s", exc)

    # ------------------------------------------------------------------
    # 自动控制任务实现
    # ------------------------------------------------------------------
    def _start_angle_task(self, target_func):
        """内部通用函数：以互斥方式启动角度类任务线程。"""

        if self.angle_task_lock.locked():
            rospy.logwarn("Angle-like task already running, ignore new request.")
            return

        thread = threading.Thread(target=target_func)
        thread.daemon = True
        thread.start()

    # ---------- 普通角度定控任务 ----------
    def start_angle_normal_task(self):
        """启动角度定控任务1（普通定控，不含 PID 轨迹切换）。

        任务内容（来自最新 ROS 端控制计划）：
        1) 让 Yaw 轴电机正转 36°，速度 36°/s；
           同时 Pitch 轴电机正转 90°，速度 180°/s；
        2) 完成后让 Yaw 轴电机反转 36°，速度 36°/s；
           同时 Pitch 轴电机反转 90°，速度 180°/s；

        假设电机速度准确，根据增量角度与速度估算运行时间（再加安全裕度），
        在每一步结束后更新内部姿态估计并写入姿态文件。
        """

        def task():
            with self.angle_task_lock:
                def run_sequence():
                    # 读取当前姿态估计，基于此计算目标角度
                    with self.attitude_lock:
                        current_yaw = self.yaw_angle
                        current_pitch = self.pitch_angle

                    # 步骤 1：Yaw +36°, Pitch +90°（非 PID 模式下为“相对角度”指令）
                    target_yaw_1 = self._wrap_angle_180(current_yaw + 36.0)
                    target_pitch_1 = self._wrap_angle_180(current_pitch + 90.0)

                    msg1 = MotorCommand()
                    # 当 PID 未启用时，TPY/TPP 表示“相对旋转角度”
                    msg1.tpy = 36.0
                    msg1.tpp = 90.0
                    msg1.tvy = 36.0
                    msg1.tvp = 180.0
                    msg1.te = 0x00
                    msg1.tpid = 0xFF
                    msg1.td = 0

                    if self.stop_auto_flag:
                        rospy.logwarn("[AngleNormal] Stop flag set before Step1, abort sequence.")
                        return

                    cmd1 = self.build_protocol_command(msg1)
                    rospy.loginfo("[AngleNormal] Step1: %s", cmd1)
                    self.send_command(cmd1)

                    # 估算运行时间并增加时间裕度 self.time_margin（秒）
                    sleep_time = max(36.0 / msg1.tvy, 90.0 / msg1.tvp) + self.time_margin
                    end_time = time.time() + sleep_time
                    while not rospy.is_shutdown() and time.time() < end_time:
                        if self.stop_auto_flag:
                            rospy.logwarn("[AngleNormal] Stop flag set during Step1, abort sequence.")
                            return
                        time.sleep(0.1)

                    # 若未被中断，则认为已到达目标姿态
                    if not self.stop_auto_flag:
                        self._set_attitude(target_yaw_1, target_pitch_1)

                    # 步骤 2：Yaw -36°, Pitch -90°（同样使用相对角度）
                    delta_yaw_2 = -36.0
                    delta_pitch_2 = -90.0
                    target_yaw_2 = self._wrap_angle_180(target_yaw_1 + delta_yaw_2)
                    target_pitch_2 = self._wrap_angle_180(target_pitch_1 + delta_pitch_2)

                    msg2 = MotorCommand()
                    msg2.tpy = delta_yaw_2
                    msg2.tpp = delta_pitch_2
                    msg2.tvy = 36.0
                    msg2.tvp = 180.0
                    msg2.te = 0x00
                    msg2.tpid = 0xFF
                    msg2.td = 0

                    if self.stop_auto_flag:
                        rospy.logwarn("[AngleNormal] Stop flag set before Step2, abort sequence.")
                        return

                    cmd2 = self.build_protocol_command(msg2)
                    rospy.loginfo("[AngleNormal] Step2: %s", cmd2)
                    self.send_command(cmd2)

                    sleep_time = max(abs(delta_yaw_2) / msg2.tvy, abs(delta_pitch_2) / msg2.tvp) + self.time_margin
                    end_time = time.time() + sleep_time
                    while not rospy.is_shutdown() and time.time() < end_time:
                        if self.stop_auto_flag:
                            rospy.logwarn("[AngleNormal] Stop flag set during Step2, abort sequence.")
                            return
                        time.sleep(0.1)

                    if not self.stop_auto_flag:
                        self._set_attitude(target_yaw_2, target_pitch_2)

                # 角度定控任务1只执行一轮
                if not rospy.is_shutdown():
                    run_sequence()
                rospy.loginfo("[AngleNormal] Sequence finished.")

        self._start_angle_task(task)

    # ---------- 角度 PID 定控任务 ----------
    def start_angle_pid_task(self):
        """启动角度 PID 定控任务。"""

        def task():
            with self.angle_task_lock:
                # 角度 PID 定控任务：
                # 1. Yaw -> -60°, Pitch -> -90°
                # 2. Yaw -> -120°, Pitch -> 0°
                # 使用 TPID=0 开启 PID 精确控制
                # 步骤 1
                msg1 = MotorCommand()
                msg1.tpy = -60.0
                msg1.tpp = -90.0
                msg1.tvy = 30.0
                msg1.tvp = 45.0
                msg1.te = 0x00
                msg1.tpid = 0  # 开启 PID
                msg1.td = 0
                if self.stop_auto_flag:
                    rospy.logwarn("[AnglePID] Stop flag set before Step1, abort sequence.")
                    return

                cmd1 = self.build_protocol_command(msg1)
                rospy.loginfo("[AnglePID] Step1: %s", cmd1)
                self.send_command(cmd1)

                # 增加等待裕度 self.time_margin，避免尚未到位就开始下一步
                sleep_time = max(abs(msg1.tpy / msg1.tvy), abs(msg1.tpp / msg1.tvp)) + self.time_margin
                end_time = time.time() + sleep_time
                while not rospy.is_shutdown() and time.time() < end_time:
                    if self.stop_auto_flag:
                        rospy.logwarn("[AnglePID] Stop flag set during Step1, abort sequence.")
                        return
                    time.sleep(0.1)

                # 若未被中断，则认为到达 (-60, -90)
                if not self.stop_auto_flag:
                    self._set_attitude(msg1.tpy, msg1.tpp)

                # 步骤 2
                msg2 = MotorCommand()
                msg2.tpy = -120.0
                msg2.tpp = 0.0
                msg2.tvy = 30.0
                msg2.tvp = 45.0
                msg2.te = 0x00
                msg2.tpid = 0
                msg2.td = 0
                if self.stop_auto_flag:
                    rospy.logwarn("[AnglePID] Stop flag set before Step2, abort sequence.")
                    return

                cmd2 = self.build_protocol_command(msg2)
                rospy.loginfo("[AnglePID] Step2: %s", cmd2)
                self.send_command(cmd2)

                # 同理，为第二步也增加时间裕度 self.time_margin
                sleep_time = max(abs((msg2.tpy - msg1.tpy) / msg2.tvy), abs((msg2.tpp - msg1.tpp) / msg2.tvp)) + self.time_margin
                end_time = time.time() + sleep_time
                while not rospy.is_shutdown() and time.time() < end_time:
                    if self.stop_auto_flag:
                        rospy.logwarn("[AnglePID] Stop flag set during Step2, abort sequence.")
                        return
                    time.sleep(0.1)

                if not self.stop_auto_flag:
                    self._set_attitude(msg2.tpy, msg2.tpp)

                rospy.loginfo("[AnglePID] Sequence finished.")

        self._start_angle_task(task)

    # ---------- 速度定控任务：Yaw/Pitch 同步周期运动 ----------
    def speed_normal_task(self):
        """速度定控任务1。

        要求：
        - Yaw：36°/s，10 秒一个周期，从 0° 转到 +180° 再回到 0°；
        - Pitch：180°/s，2 秒一个周期，路径为 0→+90→0→-90→0；
        - 时间上严格同步：Yaw 转 1 个周期，Pitch 正好转 5 个周期；
        - 每一次发送速度指令后，额外等待 self.time_margin 秒作为执行裕度。
        """

        rospy.loginfo("[SpeedNormal] Start loop (speed-control mode, continuous PWM).")

        # 依据《ROS端控制计划指导》：
        # - Yaw：36°/s，10 秒一个周期：0→+180→0（5s 正转，5s 反转）
        # - Pitch：180°/s，2 秒一个周期：0→+90→0→-90→0（每段 0.5s）
        yaw_speed_abs = 36.0
        pitch_speed_abs = 180.0

        # 关键：进入 ESP32 的“速度控制模式”需要 TPY/TPP 字段为空。
        # ROS msg 无法省略字段，这里使用 NaN 表示“无值”，由 _format_angle 格式化为空字符串。
        nan = float("nan")

        def send_speed(yaw_speed_cmd, pitch_speed_cmd):
            msg = MotorCommand()
            msg.tpy = nan
            msg.tpp = nan
            msg.tvy = float(yaw_speed_cmd)
            msg.tvp = float(pitch_speed_cmd)
            msg.te = 0x00
            msg.tpid = 0xFF
            msg.td = 0
            self.send_command(self.build_protocol_command(msg))

        last_yaw_dir = None
        last_pitch_sign = None
        t0 = time.time()
        last_time = t0

        # 先下发一次初始速度，确保电机立刻进入连续匀速输出
        send_speed(+yaw_speed_abs, +pitch_speed_abs)
        last_yaw_dir = 1.0
        last_pitch_sign = 1.0

        while not rospy.is_shutdown() and self.speed_auto_mode:
            now = time.time()
            elapsed = now - last_time
            last_time = now

            if self.stop_auto_flag:
                rospy.logwarn("[SpeedNormal] Stop flag set, abort speed NORMAL loop.")
                break

            # 以“真实时间”确定当前周期相位，减少因为 sleep 漂移导致的节拍误差
            yaw_phase = (now - t0) % 10.0
            yaw_dir = 1.0 if yaw_phase < 5.0 else -1.0

            pitch_phase = (now - t0) % 2.0
            if pitch_phase < 0.5:
                pitch_sign = 1.0
            elif pitch_phase < 1.0:
                pitch_sign = -1.0
            elif pitch_phase < 1.5:
                pitch_sign = -1.0
            else:
                pitch_sign = 1.0

            # 只有在方向段切换时才重发速度指令，避免“频繁重配 PWM”造成抖动
            if yaw_dir != last_yaw_dir or pitch_sign != last_pitch_sign:
                send_speed(yaw_dir * yaw_speed_abs, pitch_sign * pitch_speed_abs)
                last_yaw_dir = yaw_dir
                last_pitch_sign = pitch_sign

            # 姿态估计：按当前速度 * 时间增量积分（估算值，不依赖传感器）
            if elapsed > 0.0:
                self._add_attitude(yaw_dir * yaw_speed_abs * elapsed, pitch_sign * pitch_speed_abs * elapsed)

            # 小步睡眠：既保证及时响应 stop，又不会忙等
            time.sleep(0.02)

        # 退出速度模式前显式停机：否则 ESP32 会继续保持上一次速度连续输出。
        # 但若 stop_auto_flag 已置位（例如 TE=3 暂停 / 手动命令抢占），避免发送 stop 覆盖后续命令。
        if not self.stop_auto_flag:
            try:
                send_speed(0.0, 0.0)
            except Exception:
                pass

        rospy.loginfo("[SpeedNormal] Exit loop.")

    # ---------- 速度 PID 定控任务 ----------
    def speed_pid_task(self):
        """速度 PID 定控任务。

        Yaw 轴：按 -60, -120, -180(=180), 120, 60, 0 度循环；
        Pitch 轴：按 -90, 0 度循环；
        Yaw 速度 30°/s，Pitch 速度 45°/s，开启 PID（TPID=0）。
        """

        rospy.loginfo("[SpeedPID] Start loop.")
        yaw_points = [-60.0, -120.0, -180.0, 120.0, 60.0, 0.0]
        pitch_points = [-90.0, 0.0]
        yaw_idx = 0
        pitch_idx = 0

        while not rospy.is_shutdown() and self.speed_auto_mode:
            # 若收到暂停或手动优先命令，立即退出速度 PID 循环
            if self.stop_auto_flag:
                rospy.logwarn("[SpeedPID] Stop flag set, abort speed PID loop.")
                break
            msg = MotorCommand()
            msg.tpy = yaw_points[yaw_idx]
            msg.tpp = pitch_points[pitch_idx]
            msg.tvy = 30.0
            msg.tvp = 45.0
            msg.te = 0x00
            msg.tpid = 0  # 开启 PID
            msg.td = 0
            cmd = self.build_protocol_command(msg)
            rospy.loginfo("[SpeedPID] Target Yaw=%.1f, Pitch=%.1f -> %s", msg.tpy, msg.tpp, cmd)
            self.send_command(cmd)

            # 按角度/速度估算时间，并在等待期间周期检查停止标志，附加 self.time_margin
            sleep_time = max(abs(60.0 / msg.tvy), abs(90.0 / msg.tvp)) + self.time_margin
            end_time = time.time() + sleep_time
            while not rospy.is_shutdown() and time.time() < end_time:
                if self.stop_auto_flag or not self.speed_auto_mode:
                    rospy.logwarn("[SpeedPID] Stop flag or speed_auto_mode False during wait, abort loop.")
                    self.speed_auto_mode = False
                    rospy.loginfo("[SpeedPID] Exit loop.")
                    return
                time.sleep(0.1)

            # 若未被中断，则认为到达该目标姿态
            if not self.stop_auto_flag and self.speed_auto_mode:
                self._set_attitude(msg.tpy, msg.tpp)

            yaw_idx = (yaw_idx + 1) % len(yaw_points)
            pitch_idx = (pitch_idx + 1) % len(pitch_points)

        rospy.loginfo("[SpeedPID] Exit loop.")

    # ------------------------------------------------------------------
    # 串口读取与心跳
    # ------------------------------------------------------------------
    def read_serial(self):
        """后台线程：持续读取 ESP32 串口反馈并发布到 `/esp32/status`。"""

        while not rospy.is_shutdown():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        rospy.loginfo("ESP32 Response: %s", line)
                        
                        #打成 WARN，方便在 ROS 端看到MPU_STATUS
                        if line.startswith("MPU_STATUS:"):
                            rospy.logwarn("ESP32 %s", line)
                            
                        status_msg = String(data=line)
                        self.status_pub.publish(status_msg)
                else:
                    # 避免空转占用过多 CPU
                    time.sleep(0.01)
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logwarn("Error reading from serial: %s", exc)
                time.sleep(0.1)

    # ------------------------------------------------------------------
    # 键盘指令：R / K
    # ------------------------------------------------------------------
    def keyboard_loop(self):
        """在单独线程中监听终端键盘输入：R / K。

        - 输入 "R"：读取姿态数据文件的最后一条记录（或当前估计姿态），
          让电机以最短路径回到“原点”(0°,0°)。
        - 输入 "K"：清空姿态数据文件，并将当前位置设为新的原点。

        注意：该循环会阻塞标准输入，但不影响 ROS 主线程和串口线程。
        """

        # Python2/3 兼容的输入函数
        try:
            input_fn = raw_input  # type: ignore[name-defined]
        except NameError:  # Python3
            input_fn = input

        while not rospy.is_shutdown():
            try:
                cmd = input_fn("[ESP32Bridge] Enter command (R=return, K=reset origin, others=ignore): ").strip().upper()
            except EOFError:
                # 终端被关闭，无需再监听
                break
            except Exception as exc:  # pylint: disable=broad-except
                rospy.logwarn("Keyboard input error: %s", exc)
                continue

            if cmd == "R":
                self.handle_return_to_origin()
            elif cmd == "K":
                self.handle_reset_origin()
            else:
                rospy.loginfo("[ESP32Bridge] Ignore keyboard cmd: %s", cmd)

    def _read_last_attitude_from_file(self):
        """从姿态文件中读取最后一行的 (yaw, pitch)。

        如果文件不存在或为空，则返回当前内部估计姿态。
        """

        try:
            with open(self.attitude_file, "r") as f:
                lines = f.readlines()
            if not lines:
                raise IOError("empty attitude file")
            last = lines[-1].strip().split()
            if len(last) >= 3:
                yaw = float(last[1])
                pitch = float(last[2])
                return yaw, pitch
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("Failed to read last attitude from file, use current estimate instead: %s", exc)

        with self.attitude_lock:
            return self.yaw_angle, self.pitch_angle

    def handle_return_to_origin(self):
        """处理键盘 R：让电机以最短路径回到原点 (0°,0°)。"""

        current_yaw, current_pitch = self._read_last_attitude_from_file()
        rospy.loginfo("[Keyboard] Return to origin from yaw=%.1f, pitch=%.1f", current_yaw, current_pitch)

        # 计算到 0° 的最短角度差（在 [-180,180] 内）
        def shortest_delta(angle):
            delta = -angle
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            return delta

        delta_yaw = shortest_delta(current_yaw)
        delta_pitch = shortest_delta(current_pitch)

        target_yaw = self._wrap_angle_180(current_yaw + delta_yaw)  # 理论上为 0
        target_pitch = self._wrap_angle_180(current_pitch + delta_pitch)  # 理论上为 0

        msg = MotorCommand()
        # 非 PID 模式下，TPY/TPP 为“相对角度”
        msg.tpy = delta_yaw
        msg.tpp = delta_pitch
        msg.tvy = 36.0   # 使用与角度/速度任务相同的速度
        msg.tvp = 180.0
        msg.te = 0x00
        msg.tpid = 0xFF
        msg.td = 0

        cmd = self.build_protocol_command(msg)
        rospy.loginfo("[Keyboard] Return-to-origin cmd: %s", cmd)
        self.send_command(cmd)

        # 估算时间并等待执行结束，附加时间裕度 self.time_margin
        sleep_time = max(abs(delta_yaw) / msg.tvy, abs(delta_pitch) / msg.tvp) + self.time_margin
        end_time = time.time() + sleep_time
        while not rospy.is_shutdown() and time.time() < end_time:
            time.sleep(0.1)

        # 更新姿态为原点
        self._set_attitude(0.0, 0.0)
        rospy.loginfo("[Keyboard] Return-to-origin finished, attitude reset to (0,0).")

    def handle_reset_origin(self):
        """处理键盘 K：清空姿态数据并将当前位置设为原点。"""

        # 清空文件
        try:
            with open(self.attitude_file, "w") as f:
                f.write("")
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logwarn("Failed to clear attitude file %s: %s", self.attitude_file, exc)

        # 将当前估计姿态视为新的原点，并写入一条记录
        with self.attitude_lock:
            self.yaw_angle = 0.0
            self.pitch_angle = 0.0
            self._append_attitude_to_file()

        rospy.loginfo("[Keyboard] Origin reset, current attitude set to (0,0) and file cleared.")

    def run(self):
        """主循环：定期发送“PING”心跳。"""

        rate = rospy.Rate(10)  # 10 Hz
        last_ping_time = 0.0
        while not rospy.is_shutdown():
            # 每 10 秒发送一次 "PING" 心跳
            now = rospy.get_time()
            if now - last_ping_time >= 10.0:
                self.send_command("PING")
                last_ping_time = now
            rate.sleep()

    def shutdown(self):
        """节点关闭时的清理工作。"""

        # 停止速度自动任务
        self.speed_auto_mode = False

        if self.ser and self.ser.is_open:
            try:
                # 可以选择发送一个“停止”命令，这里简单发送 TE=11 表示两轴失能
                stop_msg = MotorCommand()
                stop_msg.tpy = 0.0
                stop_msg.tpp = 0.0
                stop_msg.tvy = 0.0
                stop_msg.tvp = 0.0
                stop_msg.te = 0x03
                stop_msg.td = 0
                cmd = self.build_protocol_command(stop_msg)
                self.send_command(cmd)
                time.sleep(0.5)
            finally:
                self.ser.close()


def main():
    rospy.init_node("esp32_bridge")
    bridge = ESP32Bridge()

    # 注册关闭钩子
    rospy.on_shutdown(bridge.shutdown)

    try:
        bridge.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
