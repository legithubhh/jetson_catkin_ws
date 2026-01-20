from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="esp32_motor_control",
                executable="esp32_bridge",
                name="esp32_bridge",
                output="screen",
                parameters=[
                    {
                        "port": "/dev/ttyUSB0",
                        "baudrate": 115200,
                        "timeout": 1.0,
                        "time_margin": 0.1,
                        "attitude_file": "motor_attitude.log",
                    }
                ],
            )
        ]
    )
