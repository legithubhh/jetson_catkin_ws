#!/usr/bin/env python
# -*- coding:utf-8 -*-
"""ESP32 电机速度定控任务1独立测试脚本。

根据《ROS端控制计划指导》的第 4 条：

1. 直接执行速度定控任务1；
2. 当退出执行当前程序的终端时，马上暂停电机。

本脚本仅负责通过 `/esp32/control_strategy` 话题切换到“速度定控任务1”，
实际的轨迹生成与串口下发由 `esp32_bridge.py` 中的 `speed_normal_task` 完成。
"""

import rospy

from esp32_motor_control.msg import MotorCommand


def send_pause(strategy_pub):
    """向 `/esp32/control_strategy` 发送一次暂停/停机命令（TE=3）。"""

    msg = MotorCommand()
    msg.pid_auto = 1
    msg.angle_auto = 1
    msg.speed_auto = 1
    msg.te = 3
    rospy.loginfo("[MotorControl] Send PAUSE command on shutdown.")
    strategy_pub.publish(msg)


def main():
    rospy.init_node("motor_speed_control")

    # 仅使用策略话题切换到“速度定控任务1”
    strategy_pub = rospy.Publisher("/esp32/control_strategy", MotorCommand, queue_size=10)

    rospy.sleep(2.0)  # 等待桥接节点就绪

    # 在节点关闭时确保发送一次暂停命令
    def _on_shutdown():
        try:
            send_pause(strategy_pub)
        except Exception:
            pass

    rospy.on_shutdown(_on_shutdown)

    # 发送一次策略命令：pid_auto=1 且 speed_auto=1 -> 速度定控任务1
    cmd = MotorCommand()
    cmd.pid_auto = 1
    cmd.angle_auto = 0
    cmd.speed_auto = 1
    rospy.loginfo("[MotorControl] Start Speed NORMAL task1 (continuous until node exit)...")
    strategy_pub.publish(cmd)

    # 保持节点运行，直到用户 Ctrl+C 或关闭终端
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
