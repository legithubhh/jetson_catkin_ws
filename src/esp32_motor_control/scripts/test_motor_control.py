#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""ESP32 电机控制综合测试脚本。

根据**最新**《ROS端控制计划指导》要求，依次完成：

1. 角度定控任务1（普通）：
    - 向 `/esp32/control_strategy` 发布 pid_auto=1, angle_auto=1 的命令；
    - 等待任务完成后，暂停电机 5 秒；
    - 再切换速度定控任务1运行 10 秒，随后暂停电机 5 秒。
2. 角度 PID 定控任务1：
    - 向 `/esp32/control_strategy` 发布 pid_auto=0, angle_auto=1 的命令；
    - 等待任务完成后，暂停电机 5 秒；
    - 再切换速度 PID 定控任务1运行 12 秒，随后暂停电机 5 秒。
3. 心形轨迹绘制：
    - 直接向 `/esp32/motor_cmd` 发送一系列 (Yaw, Pitch) 角度指令，
    - Pitch 角度始终限制在 [-90°, 90°] 范围内。
"""

import math

import rospy

from esp32_motor_control.msg import MotorCommand


def publish_strategy(pub, msg, desc, wait_time):
    """发送控制策略命令到 `/esp32/control_strategy`。"""

    rospy.loginfo(
        "[Test/Strategy] %s -> pid_auto=%d, angle_auto=%d, speed_auto=%d",
        desc,
        getattr(msg, "pid_auto", 1),
        msg.angle_auto,
        msg.speed_auto,
    )
    pub.publish(msg)
    rospy.sleep(wait_time)


def publish_motor(pub, msg, desc, wait_time):
    """发送普通电机命令到 `/esp32/motor_cmd`。"""

    rospy.loginfo(
        "[Test/Motor] %s -> TPY=%.1f, TPP=%.1f, TVY=%.1f, TVP=%.1f, TE=%d, TPID=%d, TD=%d",
        desc,
        msg.tpy,
        msg.tpp,
        msg.tvy,
        msg.tvp,
        msg.te,
        msg.tpid,
        msg.td,
    )
    pub.publish(msg)
    rospy.sleep(wait_time)


def send_pause(strategy_pub):
    """向 `/esp32/control_strategy` 发送一次暂停/停机命令。

    在桥接节点中约定：当 te == 3 时，立即：
    - 置位 stop_auto_flag, speed_auto_mode=False；
    - 向 ESP32 下发 TE=3 的停机指令；
    从而真实停止当前自动任务与电机运动。
    """

    msg = MotorCommand()
    msg.pid_auto = 1
    msg.angle_auto = 1
    msg.speed_auto = 1
    msg.te = 3  # 关键字段：暂停/停机
    rospy.loginfo("[Test] ===== Send PAUSE command =====")
    publish_strategy(strategy_pub, msg, "PAUSE", 1.0)


def run_angle_normal_and_speed_normal(strategy_pub):
    """1) 角度定控任务1 -> 暂停 5s -> 速度定控任务1 10s -> 暂停 5s。"""

    rospy.loginfo("[Test] ===== Angle NORMAL task =====")
    msg = MotorCommand()
    # pid_auto=1 且 angle_auto=1 -> 角度定控任务1
    msg.pid_auto = 1
    msg.angle_auto = 1
    msg.speed_auto = 1  # 两者都为 1 时仍视为角度任务
    publish_strategy(strategy_pub, msg, "Angle normal", 1.0)

    # 等待一次角度定控任务1完成（两步，预估 <10s，这里取 10s 裕度）
    rospy.sleep(10.0)
    # 先真实下发暂停命令，再额外等待 5s 观察
    send_pause(strategy_pub)
    rospy.loginfo("[Test] ===== Angle NORMAL done, pause 5s =====")
    rospy.sleep(5.0)

    rospy.loginfo("[Test] ===== Speed NORMAL task (10s) =====")
    msg = MotorCommand()
    # pid_auto=1 且 speed_auto=1 -> 速度定控任务1
    msg.pid_auto = 1
    msg.angle_auto = 0
    msg.speed_auto = 1
    publish_strategy(strategy_pub, msg, "Speed normal", 1.0)

    # 速度定控任务1按 10 秒运行
    rospy.sleep(10.0)
    send_pause(strategy_pub)
    rospy.loginfo("[Test] ===== Speed NORMAL done, pause 5s =====")
    rospy.sleep(5.0)


def run_angle_pid_and_speed_pid(strategy_pub):
    """2) 角度 PID 定控任务1 -> 暂停 5s -> 速度 PID 定控任务1 12s -> 暂停 5s。"""

    rospy.loginfo("[Test] ===== Angle PID task =====")
    msg = MotorCommand()
    # pid_auto=0 且 angle_auto=1 -> 角度 PID 定控任务1
    msg.pid_auto = 0
    msg.angle_auto = 1
    msg.speed_auto = 1
    publish_strategy(strategy_pub, msg, "Angle PID", 1.0)

    # 角度 PID 任务1包含两步，预估 <10s，这里取 10s 裕度
    rospy.sleep(10.0)
    send_pause(strategy_pub)
    rospy.loginfo("[Test] ===== Angle PID done, pause 5s =====")
    rospy.sleep(5.0)

    rospy.loginfo("[Test] ===== Speed PID task (12s) =====")
    msg = MotorCommand()
    # pid_auto=0 且 speed_auto=1 -> 速度 PID 定控任务1
    msg.pid_auto = 0
    msg.angle_auto = 0
    msg.speed_auto = 1
    publish_strategy(strategy_pub, msg, "Speed PID", 1.0)

    # 速度 PID 任务1按 12 秒运行
    rospy.sleep(12.0)
    send_pause(strategy_pub)
    rospy.loginfo("[Test] ===== Speed PID done, pause 5s =====")
    rospy.sleep(5.0)


def run_heart_trajectory(pub):
    """让电机绘制一个“心形”轨迹。

    采用参数方程心形曲线：
        x(t) = 16 sin^3(t)
        y(t) = 13 cos(t) - 5 cos(2t) - 2 cos(3t) - cos(4t)

    这里简单地将：
        - Yaw 轴 (TPY) 映射到 x(t) 的某个缩放；
        - Pitch 轴 (TPP) 映射到 y(t) 的某个缩放，且强制裁剪在 [-90°, 90°] 范围内。
    """

    rospy.loginfo("[Test] ===== Start heart trajectory =====")

    # 采样步数，越大路径越平滑
    num_samples = 80
    # 参数 t 从 0 到 2π
    t_min, t_max = 0.0, 2.0 * math.pi

    for i in range(num_samples + 1):
        if rospy.is_shutdown():
            break

        t = t_min + (t_max - t_min) * float(i) / float(num_samples)

        # 经典心形参数方程
        x = 16.0 * math.sin(t) ** 3
        y = (
            13.0 * math.cos(t)
            - 5.0 * math.cos(2.0 * t)
            - 2.0 * math.cos(3.0 * t)
            - 1.0 * math.cos(4.0 * t)
        )

        # 将 (x, y) 映射到电机角度，适当缩放到合理范围
        # 这里 yaw 使用较小缩放，pitch 使用较大缩放，以形成明显心形
        yaw_angle = x * 3.0          # 约在 [-48°, 48°]
        pitch_angle = y * 2.0        # 原始 y 范围约 [-17, 21]，乘 2 后约 [-34°, 42°]

        # 再次确保 Pitch 在 [-90°, 90°] 范围内
        pitch_angle = max(-90.0, min(90.0, pitch_angle))

        msg = MotorCommand()
        msg.angle_auto = 1
        msg.speed_auto = 1
        msg.tpy = yaw_angle
        msg.tpp = pitch_angle

        # 设置较高的速度，使轨迹连贯，但不至于过快
        msg.tvy = 120.0
        msg.tvp = 120.0
        msg.te = 0x00  # 两轴使能
        msg.tpid = 0xFF
        msg.td = 0

        # 每个点之间间隔 0.15 秒左右，整体约 12 秒
        publish_motor(pub, msg, "Heart point %d/%d" % (i, num_samples), 0.15)

    rospy.loginfo("[Test] ===== Heart trajectory finished =====")


def main():
    rospy.init_node("motor_test")

    # 策略发布者：用于触发各种自动/ PID 任务
    strategy_pub = rospy.Publisher("/esp32/control_strategy", MotorCommand, queue_size=10)
    # 电机发布者：用于心形轨迹等普通控制
    motor_pub = rospy.Publisher("/esp32/motor_cmd", MotorCommand, queue_size=10)

    # 等待桥接节点和话题连接就绪
    rospy.sleep(2.0)

    # 在节点退出时自动发送暂停命令，满足“退出终端马上暂停电机”的需求
    def _on_shutdown():
        try:
            send_pause(strategy_pub)
        except Exception:
            pass

    rospy.on_shutdown(_on_shutdown)

    # 1) 角度定控任务1 + 速度定控任务1
    run_angle_normal_and_speed_normal(strategy_pub)

    # 2) 角度 PID 定控任务1 + 速度 PID 定控任务1
    # run_angle_pid_and_speed_pid(strategy_pub)

    # 3) 心形轨迹
    # run_heart_trajectory(motor_pub)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
