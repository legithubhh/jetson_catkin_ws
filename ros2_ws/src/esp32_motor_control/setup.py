from setuptools import find_packages, setup

package_name = "esp32_motor_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/esp32_bridge.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@todo.todo",
    description="ROS 2 rclpy bridge node for ESP32 motor control over serial.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "esp32_bridge = esp32_motor_control.esp32_bridge_node:main",
            "motor_control = esp32_motor_control.motor_control:main",
            "test_motor_control = esp32_motor_control.test_motor_control:main",
        ],
    },
)
