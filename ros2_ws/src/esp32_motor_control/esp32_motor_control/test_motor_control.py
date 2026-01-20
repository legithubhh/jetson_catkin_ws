import math
import time

import rclpy
from rclpy.node import Node

from esp32_motor_control_msgs.msg import MotorCommand


class TestMotorControlNode(Node):
    """ROS2 replacement for ROS1 test_motor_control.py.

    This script triggers tasks via `/esp32/control_strategy` and can also publish manual points
    to `/esp32/motor_cmd`.
    """

    def __init__(self) -> None:
        super().__init__("motor_test")
        self.strategy_pub = self.create_publisher(MotorCommand, "/esp32/control_strategy", 10)
        self.motor_pub = self.create_publisher(MotorCommand, "/esp32/motor_cmd", 10)

    def publish_strategy(self, msg: MotorCommand, desc: str, wait_s: float) -> None:
        self.get_logger().info(
            f"[Test/Strategy] {desc} -> pid_auto={int(msg.pid_auto)}, angle_auto={int(msg.angle_auto)}, speed_auto={int(msg.speed_auto)}, te={int(msg.te)}"
        )
        self.strategy_pub.publish(msg)
        time.sleep(wait_s)

    def publish_motor(self, msg: MotorCommand, desc: str, wait_s: float) -> None:
        self.get_logger().info(
            f"[Test/Motor] {desc} -> TPY={msg.tpy:.1f}, TPP={msg.tpp:.1f}, TVY={msg.tvy:.1f}, TVP={msg.tvp:.1f}, TE={int(msg.te)}, TPID={int(msg.tpid)}, TD={int(msg.td)}"
        )
        self.motor_pub.publish(msg)
        time.sleep(wait_s)

    def send_pause(self) -> None:
        msg = MotorCommand()
        msg.pid_auto = 1
        msg.angle_auto = 1
        msg.speed_auto = 1
        msg.te = 3
        self.get_logger().info("[Test] ===== Send PAUSE (TE=3) =====")
        self.publish_strategy(msg, "PAUSE", 1.0)

    def run_angle_normal_and_speed_normal(self) -> None:
        self.get_logger().info("[Test] ===== Angle NORMAL task =====")
        msg = MotorCommand()
        msg.pid_auto = 1
        msg.angle_auto = 1
        msg.speed_auto = 1
        self.publish_strategy(msg, "Angle normal", 1.0)

        time.sleep(10.0)
        self.send_pause()
        time.sleep(5.0)

        self.get_logger().info("[Test] ===== Speed NORMAL task (10s) =====")
        msg = MotorCommand()
        msg.pid_auto = 1
        msg.angle_auto = 0
        msg.speed_auto = 1
        self.publish_strategy(msg, "Speed normal", 1.0)

        time.sleep(10.0)
        self.send_pause()
        time.sleep(5.0)

    def run_angle_pid_and_speed_pid(self) -> None:
        self.get_logger().info("[Test] ===== Angle PID task =====")
        msg = MotorCommand()
        msg.pid_auto = 0
        msg.angle_auto = 1
        msg.speed_auto = 1
        self.publish_strategy(msg, "Angle PID", 1.0)

        time.sleep(10.0)
        self.send_pause()
        time.sleep(5.0)

        self.get_logger().info("[Test] ===== Speed PID task (12s) =====")
        msg = MotorCommand()
        msg.pid_auto = 0
        msg.angle_auto = 0
        msg.speed_auto = 1
        self.publish_strategy(msg, "Speed PID", 1.0)

        time.sleep(12.0)
        self.send_pause()
        time.sleep(5.0)

    def run_heart_trajectory(self) -> None:
        self.get_logger().info("[Test] ===== Start heart trajectory =====")

        num_samples = 80
        t_min, t_max = 0.0, 2.0 * math.pi

        for i in range(num_samples + 1):
            if not rclpy.ok():
                break

            t = t_min + (t_max - t_min) * float(i) / float(num_samples)

            x = 16.0 * math.sin(t) ** 3
            y = 13.0 * math.cos(t) - 5.0 * math.cos(2.0 * t) - 2.0 * math.cos(3.0 * t) - 1.0 * math.cos(4.0 * t)

            yaw_angle = x * 3.0
            pitch_angle = max(-90.0, min(90.0, y * 2.0))

            msg = MotorCommand()
            msg.angle_auto = 0
            msg.speed_auto = 0
            msg.tpy = yaw_angle
            msg.tpp = pitch_angle
            msg.tvy = 120.0
            msg.tvp = 120.0
            msg.te = 0x00
            msg.tpid = 0xFF
            msg.td = 0

            self.publish_motor(msg, f"Heart point {i}/{num_samples}", 0.15)

        self.get_logger().info("[Test] ===== Heart trajectory finished =====")


def main() -> None:
    rclpy.init()
    node = TestMotorControlNode()

    try:
        time.sleep(2.0)
        node.run_angle_normal_and_speed_normal()
        # node.run_angle_pid_and_speed_pid()
        # node.run_heart_trajectory()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_pause()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
