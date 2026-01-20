import rclpy
from rclpy.node import Node

from esp32_motor_control_msgs.msg import MotorCommand


class MotorControlNode(Node):
    """ROS2 replacement for the ROS1 motor_control.py.

    Starts Speed NORMAL task1 via `/esp32/control_strategy` and sends TE=3 pause on shutdown.
    """

    def __init__(self) -> None:
        super().__init__("motor_speed_control")
        self.strategy_pub = self.create_publisher(MotorCommand, "/esp32/control_strategy", 10)

    def start_speed_normal(self) -> None:
        msg = MotorCommand()
        msg.pid_auto = 1
        msg.angle_auto = 0
        msg.speed_auto = 1
        self.get_logger().info("Start Speed NORMAL task1 (until node exit)...")
        self.strategy_pub.publish(msg)

    def send_pause(self) -> None:
        msg = MotorCommand()
        msg.pid_auto = 1
        msg.angle_auto = 1
        msg.speed_auto = 1
        msg.te = 3
        self.get_logger().info("Send PAUSE (TE=3) on shutdown.")
        self.strategy_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = MotorControlNode()

    try:
        # Wait a bit for the bridge to be ready.
        node.create_timer(2.0, node.start_speed_normal)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send_pause()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
