import math
import threading
import time
from typing import Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import serial
from std_msgs.msg import String

from esp32_motor_control_msgs.msg import MotorCommand


class ESP32BridgeNode(Node):
    """ROS2 ↔ ESP32 bridge.

    - Subscribes:
      - `/esp32/motor_cmd` (MotorCommand): manual commands
      - `/esp32/control_strategy` (MotorCommand): start/stop built-in tasks
    - Publishes:
      - `/esp32/status` (std_msgs/String): raw serial lines from ESP32

    Serial protocol format (one line per command):
      TPY:{tpy} TPP:{tpp} TVY:{tvy} TVP:{tvp} TE:{te:02d} TPID:{tpid:02d} TD:{td:02d}\n
    Convention in this project:
    - TE==3 on `/esp32/control_strategy` means pause/stop now.
    - angle_auto==1 / speed_auto==1 are task triggers (source of truth).
    """

    def __init__(self) -> None:
        super().__init__("esp32_bridge")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("timeout", 1.0)
        self.declare_parameter("time_margin", 0.5)
        self.declare_parameter("attitude_file", "motor_attitude.log")

        self.port: str = self.get_parameter("port").value
        self.baudrate: int = int(self.get_parameter("baudrate").value)
        self.timeout: float = float(self.get_parameter("timeout").value)
        self.time_margin: float = max(0.0, float(self.get_parameter("time_margin").value))
        self.attitude_file: str = str(self.get_parameter("attitude_file").value)

        self.speed_auto_mode = False
        self.angle_task_lock = threading.Lock()
        self.stop_auto_flag = False

        self.yaw_angle = 0.0
        self.pitch_angle = 0.0
        self.attitude_lock = threading.Lock()

        self.ser: Optional[serial.Serial] = None
        self.connect_serial()

        self.status_pub = self.create_publisher(String, "/esp32/status", 10)
        self.strategy_sub = self.create_subscription(
            MotorCommand, "/esp32/control_strategy", self.control_strategy_callback, 10
        )
        self.cmd_sub = self.create_subscription(MotorCommand, "/esp32/motor_cmd", self.motor_cmd_callback, 10)

        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()

        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()

        self.ping_timer = self.create_timer(10.0, self._send_ping)

        self.get_logger().info(f"ESP32 Bridge initialized on port {self.port}")

    # ---------------- attitude helpers ----------------
    @staticmethod
    def _wrap_angle_180(angle: float) -> float:
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle

    def _append_attitude_to_file(self) -> None:
        try:
            with open(self.attitude_file, "a", encoding="utf-8") as f:
                f.write("{:.3f} {:.3f} {:.3f}\n".format(time.time(), self.yaw_angle, self.pitch_angle))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to write attitude file {self.attitude_file}: {exc}")

    def _set_attitude(self, yaw_deg: float, pitch_deg: float) -> None:
        with self.attitude_lock:
            self.yaw_angle = self._wrap_angle_180(float(yaw_deg))
            self.pitch_angle = self._wrap_angle_180(float(pitch_deg))
            self._append_attitude_to_file()

    def _add_attitude(self, delta_yaw_deg: float, delta_pitch_deg: float) -> None:
        with self.attitude_lock:
            self.yaw_angle = self._wrap_angle_180(self.yaw_angle + float(delta_yaw_deg))
            self.pitch_angle = self._wrap_angle_180(self.pitch_angle + float(delta_pitch_deg))
            self._append_attitude_to_file()

    # ---------------- serial ----------------
    def connect_serial(self) -> None:
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=self.timeout,
            )
            self.get_logger().info(f"Successfully connected to ESP32 on {self.port}")
            time.sleep(2.0)
            self._read_initial_message()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Serial connection error: {exc}")
            raise

    def _read_initial_message(self) -> None:
        try:
            if self.ser and self.ser.in_waiting > 0:
                initial_msg = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if initial_msg:
                    self.get_logger().info(f"ESP32: {initial_msg}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to read initial message: {exc}")

    def send_command(self, command: str) -> None:
        try:
            if self.ser and self.ser.is_open:
                self.ser.write((command + "\n").encode("utf-8"))
                self.ser.flush()
            else:
                self.get_logger().warn("Serial port not open, attempting reconnect...")
                self.connect_serial()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Error sending command: {exc}")

    def read_serial(self) -> None:
        while rclpy.ok():
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self.get_logger().info(f"ESP32 Response: {line}")
                        if line.startswith("MPU_STATUS:"):
                            self.get_logger().warn(f"ESP32 {line}")
                        self.status_pub.publish(String(data=line))
                else:
                    time.sleep(0.01)
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Error reading from serial: {exc}")
                time.sleep(0.1)

    # ---------------- protocol ----------------
    @staticmethod
    def _format_angle(value: float) -> str:
        """Format numeric field for protocol. NaN/inf => empty string (omit field)."""
        try:
            v = float(value)
            if math.isnan(v) or math.isinf(v):
                return ""
            return "{:.1f}".format(v)
        except Exception:  # noqa: BLE001
            return ""

    def build_protocol_command(self, msg: MotorCommand) -> str:
        tpy = self._format_angle(msg.tpy)
        tpp = self._format_angle(msg.tpp)
        tvy = self._format_angle(msg.tvy)
        tvp = self._format_angle(msg.tvp)

        te = int(msg.te) & 0x03
        tpid = int(msg.tpid) & 0xFF
        td = int(msg.td) & 0xFF

        return (
            "TPY:{tpy} TPP:{tpp} TVY:{tvy} TVP:{tvp} "
            "TE:{te:02d} TPID:{tpid:02d} TD:{td:02d}"
        ).format(tpy=tpy, tpp=tpp, tvy=tvy, tvp=tvp, te=te, tpid=tpid, td=td)

    # ---------------- ROS callbacks ----------------
    def control_strategy_callback(self, msg: MotorCommand) -> None:
        if int(msg.te) == 3:
            self.get_logger().warn("[ESP32Bridge] Received PAUSE command from strategy.")
            self.stop_auto_flag = True
            self.speed_auto_mode = False

            stop_msg = MotorCommand()
            stop_msg.tpy = 0.0
            stop_msg.tpp = 0.0
            stop_msg.tvy = 0.0
            stop_msg.tvp = 0.0
            stop_msg.te = 3
            stop_msg.tpid = 0xFF
            stop_msg.td = 0
            self.send_command(self.build_protocol_command(stop_msg))
            return

        pid_mode_is_pid = (int(msg.pid_auto) == 0)

        if not pid_mode_is_pid:
            if int(msg.angle_auto) == 1:
                self.stop_auto_flag = False
                self.speed_auto_mode = False
                self.get_logger().info("[ESP32Bridge] Strategy: angle NORMAL task1.")
                self.start_angle_normal_task()
                return

            if int(msg.speed_auto) == 1:
                self.stop_auto_flag = False
                if not self.speed_auto_mode:
                    self.speed_auto_mode = True
                    self.get_logger().info("[ESP32Bridge] Strategy: speed NORMAL task1.")
                    threading.Thread(target=self.speed_normal_task, daemon=True).start()
                return
        else:
            if int(msg.angle_auto) == 1:
                self.stop_auto_flag = False
                self.speed_auto_mode = False
                self.get_logger().info("[ESP32Bridge] Strategy: angle PID task1.")
                self.start_angle_pid_task()
                return

            if int(msg.speed_auto) == 1:
                self.stop_auto_flag = False
                if not self.speed_auto_mode:
                    self.speed_auto_mode = True
                    self.get_logger().info("[ESP32Bridge] Strategy: speed PID task1.")
                    threading.Thread(target=self.speed_pid_task, daemon=True).start()
                return

        self.get_logger().info("[ESP32Bridge] Strategy: normal mode (wait /esp32/motor_cmd).")
        self.stop_auto_flag = False
        self.speed_auto_mode = False

    def motor_cmd_callback(self, msg: MotorCommand) -> None:
        if self.speed_auto_mode or self.angle_task_lock.locked():
            self.get_logger().warn("[ESP32Bridge] Auto task running, MANUAL cmd has higher priority -> stop auto.")
            self.stop_auto_flag = True
            self.speed_auto_mode = False

        cmd_str = self.build_protocol_command(msg)
        self.get_logger().info(f"[ESP32Bridge] Send manual cmd: {cmd_str}")
        self.send_command(cmd_str)

    # ---------------- tasks ----------------
    def _start_angle_task(self, target_func) -> None:
        if self.angle_task_lock.locked():
            self.get_logger().warn("Angle-like task already running, ignore new request.")
            return
        threading.Thread(target=target_func, daemon=True).start()

    def start_angle_normal_task(self) -> None:
        def task():
            with self.angle_task_lock:
                with self.attitude_lock:
                    current_yaw = self.yaw_angle
                    current_pitch = self.pitch_angle

                target_yaw_1 = self._wrap_angle_180(current_yaw + 36.0)
                target_pitch_1 = self._wrap_angle_180(current_pitch + 90.0)

                msg1 = MotorCommand()
                msg1.tpy = 36.0
                msg1.tpp = 90.0
                msg1.tvy = 36.0
                msg1.tvp = 180.0
                msg1.te = 0x00
                msg1.tpid = 0xFF
                msg1.td = 0

                if self.stop_auto_flag:
                    self.get_logger().warn("[AngleNormal] Stop flag set before Step1, abort.")
                    return

                self.send_command(self.build_protocol_command(msg1))

                sleep_time = max(36.0 / msg1.tvy, 90.0 / msg1.tvp) + self.time_margin
                end_time = time.time() + sleep_time
                while rclpy.ok() and time.time() < end_time:
                    if self.stop_auto_flag:
                        self.get_logger().warn("[AngleNormal] Stop flag set during Step1, abort.")
                        return
                    time.sleep(0.1)

                if not self.stop_auto_flag:
                    self._set_attitude(target_yaw_1, target_pitch_1)

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
                    self.get_logger().warn("[AngleNormal] Stop flag set before Step2, abort.")
                    return

                self.send_command(self.build_protocol_command(msg2))

                sleep_time = max(abs(delta_yaw_2) / msg2.tvy, abs(delta_pitch_2) / msg2.tvp) + self.time_margin
                end_time = time.time() + sleep_time
                while rclpy.ok() and time.time() < end_time:
                    if self.stop_auto_flag:
                        self.get_logger().warn("[AngleNormal] Stop flag set during Step2, abort.")
                        return
                    time.sleep(0.1)

                if not self.stop_auto_flag:
                    self._set_attitude(target_yaw_2, target_pitch_2)

                self.get_logger().info("[AngleNormal] Sequence finished.")

        self._start_angle_task(task)

    def start_angle_pid_task(self) -> None:
        def task():
            with self.angle_task_lock:
                msg1 = MotorCommand()
                msg1.tpy = -60.0
                msg1.tpp = -90.0
                msg1.tvy = 30.0
                msg1.tvp = 45.0
                msg1.te = 0x00
                msg1.tpid = 0
                msg1.td = 0

                if self.stop_auto_flag:
                    self.get_logger().warn("[AnglePID] Stop flag set before Step1, abort.")
                    return

                self.send_command(self.build_protocol_command(msg1))

                sleep_time = max(abs(msg1.tpy / msg1.tvy), abs(msg1.tpp / msg1.tvp)) + self.time_margin
                end_time = time.time() + sleep_time
                while rclpy.ok() and time.time() < end_time:
                    if self.stop_auto_flag:
                        self.get_logger().warn("[AnglePID] Stop flag set during Step1, abort.")
                        return
                    time.sleep(0.1)

                if not self.stop_auto_flag:
                    self._set_attitude(msg1.tpy, msg1.tpp)

                msg2 = MotorCommand()
                msg2.tpy = -120.0
                msg2.tpp = 0.0
                msg2.tvy = 30.0
                msg2.tvp = 45.0
                msg2.te = 0x00
                msg2.tpid = 0
                msg2.td = 0

                if self.stop_auto_flag:
                    self.get_logger().warn("[AnglePID] Stop flag set before Step2, abort.")
                    return

                self.send_command(self.build_protocol_command(msg2))

                sleep_time = max(
                    abs((msg2.tpy - msg1.tpy) / msg2.tvy), abs((msg2.tpp - msg1.tpp) / msg2.tvp)
                ) + self.time_margin
                end_time = time.time() + sleep_time
                while rclpy.ok() and time.time() < end_time:
                    if self.stop_auto_flag:
                        self.get_logger().warn("[AnglePID] Stop flag set during Step2, abort.")
                        return
                    time.sleep(0.1)

                if not self.stop_auto_flag:
                    self._set_attitude(msg2.tpy, msg2.tpp)

                self.get_logger().info("[AnglePID] Sequence finished.")

        self._start_angle_task(task)

    def speed_normal_task(self) -> None:
        """Speed task 1 (continuous PWM speed-control mode).

        Per project spec:
        - Yaw: 36 deg/s, 10s period, 0→+180→0 (5s forward, 5s reverse)
        - Pitch: 180 deg/s, 2s period, 0→+90→0→-90→0 (0.5s per segment)

        Implementation uses ESP32 "speed control mode": omit TPY/TPP fields and only send TVY/TVP.
        """

        self.get_logger().info("[SpeedNormal] Start loop (speed-control mode, continuous PWM).")

        yaw_speed_abs = 36.0
        pitch_speed_abs = 180.0
        nan = float("nan")

        def send_speed(yaw_speed_cmd: float, pitch_speed_cmd: float) -> None:
            out = MotorCommand()
            out.tpy = nan
            out.tpp = nan
            out.tvy = float(yaw_speed_cmd)
            out.tvp = float(pitch_speed_cmd)
            out.te = 0x00
            out.tpid = 0xFF
            out.td = 0
            self.send_command(self.build_protocol_command(out))

        t0 = time.time()
        last_time = t0
        last_yaw_dir = None
        last_pitch_sign = None

        send_speed(+yaw_speed_abs, +pitch_speed_abs)
        last_yaw_dir = 1.0
        last_pitch_sign = 1.0

        while rclpy.ok() and self.speed_auto_mode:
            now = time.time()
            elapsed = now - last_time
            last_time = now

            if self.stop_auto_flag:
                self.get_logger().warn("[SpeedNormal] Stop flag set, abort loop.")
                break

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

            if yaw_dir != last_yaw_dir or pitch_sign != last_pitch_sign:
                send_speed(yaw_dir * yaw_speed_abs, pitch_sign * pitch_speed_abs)
                last_yaw_dir = yaw_dir
                last_pitch_sign = pitch_sign

            if elapsed > 0.0:
                self._add_attitude(yaw_dir * yaw_speed_abs * elapsed, pitch_sign * pitch_speed_abs * elapsed)

            time.sleep(0.02)

        if not self.stop_auto_flag:
            try:
                send_speed(0.0, 0.0)
            except Exception:  # noqa: BLE001
                pass

        self.get_logger().info("[SpeedNormal] Exit loop.")

    def speed_pid_task(self) -> None:
        self.get_logger().info("[SpeedPID] Start loop.")
        yaw_points = [-60.0, -120.0, -180.0, 120.0, 60.0, 0.0]
        pitch_points = [-90.0, 0.0]
        yaw_idx = 0
        pitch_idx = 0

        while rclpy.ok() and self.speed_auto_mode:
            if self.stop_auto_flag:
                self.get_logger().warn("[SpeedPID] Stop flag set, abort loop.")
                break

            msg = MotorCommand()
            msg.tpy = yaw_points[yaw_idx]
            msg.tpp = pitch_points[pitch_idx]
            msg.tvy = 30.0
            msg.tvp = 45.0
            msg.te = 0x00
            msg.tpid = 0
            msg.td = 0
            self.send_command(self.build_protocol_command(msg))

            sleep_time = max(abs(60.0 / msg.tvy), abs(90.0 / msg.tvp)) + self.time_margin
            end_time = time.time() + sleep_time
            while rclpy.ok() and time.time() < end_time:
                if self.stop_auto_flag or not self.speed_auto_mode:
                    self.get_logger().warn("[SpeedPID] Stop flag or mode off during wait, abort.")
                    self.speed_auto_mode = False
                    return
                time.sleep(0.1)

            if not self.stop_auto_flag and self.speed_auto_mode:
                self._set_attitude(msg.tpy, msg.tpp)

            yaw_idx = (yaw_idx + 1) % len(yaw_points)
            pitch_idx = (pitch_idx + 1) % len(pitch_points)

        self.get_logger().info("[SpeedPID] Exit loop.")

    # ---------------- heartbeat + keyboard ----------------
    def _send_ping(self) -> None:
        try:
            self.send_command("PING")
        except Exception:  # noqa: BLE001
            pass

    def _read_last_attitude_from_file(self) -> Tuple[float, float]:
        try:
            with open(self.attitude_file, "r", encoding="utf-8") as f:
                lines = f.readlines()
            if not lines:
                raise IOError("empty attitude file")
            last = lines[-1].strip().split()
            if len(last) >= 3:
                return float(last[1]), float(last[2])
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to read last attitude, use current estimate: {exc}")

        with self.attitude_lock:
            return self.yaw_angle, self.pitch_angle

    def handle_return_to_origin(self) -> None:
        current_yaw, current_pitch = self._read_last_attitude_from_file()
        self.get_logger().info(f"[Keyboard] Return to origin from yaw={current_yaw:.1f}, pitch={current_pitch:.1f}")

        def shortest_delta(angle: float) -> float:
            delta = -angle
            if delta > 180.0:
                delta -= 360.0
            elif delta < -180.0:
                delta += 360.0
            return delta

        delta_yaw = shortest_delta(current_yaw)
        delta_pitch = shortest_delta(current_pitch)

        msg = MotorCommand()
        msg.tpy = float(delta_yaw)
        msg.tpp = float(delta_pitch)
        msg.tvy = 36.0
        msg.tvp = 180.0
        msg.te = 0x00
        msg.tpid = 0xFF
        msg.td = 0

        cmd = self.build_protocol_command(msg)
        self.get_logger().info(f"[Keyboard] Return-to-origin cmd: {cmd}")
        self.send_command(cmd)

        sleep_time = max(abs(delta_yaw) / msg.tvy, abs(delta_pitch) / msg.tvp) + self.time_margin
        time.sleep(max(0.0, sleep_time))

        self._set_attitude(0.0, 0.0)
        self.get_logger().info("[Keyboard] Return-to-origin finished, attitude reset to (0,0).")

    def handle_reset_origin(self) -> None:
        try:
            with open(self.attitude_file, "w", encoding="utf-8") as f:
                f.write("")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Failed to clear attitude file {self.attitude_file}: {exc}")

        with self.attitude_lock:
            self.yaw_angle = 0.0
            self.pitch_angle = 0.0
            self._append_attitude_to_file()

        self.get_logger().info("[Keyboard] Origin reset to (0,0) and file cleared.")

    def keyboard_loop(self) -> None:
        while rclpy.ok():
            try:
                cmd = input("[ESP32Bridge] Enter command (R=return, K=reset origin): ").strip().upper()
            except EOFError:
                break
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"Keyboard input error: {exc}")
                continue

            if cmd == "R":
                self.handle_return_to_origin()
            elif cmd == "K":
                self.handle_reset_origin()
            else:
                self.get_logger().info(f"Ignore keyboard cmd: {cmd}")

    def shutdown(self) -> None:
        self.speed_auto_mode = False

        try:
            if self.ser and self.ser.is_open:
                stop_msg = MotorCommand()
                stop_msg.tpy = 0.0
                stop_msg.tpp = 0.0
                stop_msg.tvy = 0.0
                stop_msg.tvp = 0.0
                stop_msg.te = 0x03
                stop_msg.tpid = 0xFF
                stop_msg.td = 0
                self.send_command(self.build_protocol_command(stop_msg))
                time.sleep(0.2)
        finally:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:  # noqa: BLE001
                pass


def main() -> None:
    rclpy.init()
    node = ESP32BridgeNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
