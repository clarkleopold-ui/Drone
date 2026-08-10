# AmbatuDrone controller bridge and dashboard
# PS4 controller -> Python app -> USB serial/radio bridge -> flight controller

import json
import math
import queue
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import pygame
import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200
DEFAULT_HZ = 25.0
BRIDGE_STARTUP_TIMEOUT_S = 12.0
TELEMETRY_TIMEOUT_S = 1.0
SERIAL_BUFFER_LIMIT = 4096
APP_NAME = "AmbatuDrone"
APP_VERSION = "2026.08.09"

# SDL's standard DualShock/Xbox-style controller layout.
# Raw controller debug remains available in the UI for driver-specific checks.
AXIS_LX = 0
AXIS_LY = 1
AXIS_RX = 2
AXIS_RY = 3

BUTTON_CROSS = 0
BUTTON_CIRCLE = 1
BUTTON_SQUARE = 2
BUTTON_TRIANGLE = 3
BUTTON_L1 = 9
BUTTON_R1 = 10


# -----------------------------
# Helper functions
# -----------------------------
def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_axis(raw, deadzone=0.08):
    """Convert pygame joystick axis float [-1.0, 1.0] to int [-100, 100]."""
    if abs(raw) < deadzone:
        return 0
    return clamp(int(raw * 100), -100, 100)


def get_axis_safe(joystick, axis_index, default=0.0):
    """Safely read a joystick axis without crashing if the axis does not exist."""
    if axis_index < joystick.get_numaxes():
        return joystick.get_axis(axis_index)
    return default


def get_button_safe(joystick, button_index, default=0):
    """Safely read a joystick button without crashing if the button does not exist."""
    if button_index < joystick.get_numbuttons():
        return int(joystick.get_button(button_index))
    return default


def get_snapshot_value(values, index, default=0):
    """Safely read an axis or button from a controller snapshot."""
    if 0 <= index < len(values):
        return values[index]
    return default


def safety_payload():
    """Final safe command: motor disabled, kill active, servos centered."""
    return {
        "lx": 0,
        "ly": 0,
        "rx": 0,
        "ry": 0,
        "cross": 1,
        "circle": 0,
        "square": 0,
        "triangle": 0,
        "l1": 1,
        "r1": 0,
    }


# -----------------------------
# ACK parser
# -----------------------------
def parse_ack(line):
    if not line.startswith("ACK "):
        raise ValueError("Not an AmbatuDrone ACK line")

    values = {}

    # Default values
    for motor_num in range(1, 5):
        values[f"motor{motor_num}_pwm"] = 1000
        values[f"motor{motor_num}_dir"] = "N/A"

    for key in ["servo1", "servo2", "servo3", "servo4", "lx", "ly", "rx", "ry"]:
        values[key] = 0

    values["roll_deg"] = 0.0
    values["pitch_deg"] = 0.0
    values["yaw_deg"] = 0.0

    # Parse motor_pwm=1400 1400 1400 1400
    motor_match = re.search(
        r"motor_pwm=(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)",
        line
    )
    if motor_match:
        values["motor1_pwm"] = int(motor_match.group(1))
        values["motor2_pwm"] = int(motor_match.group(2))
        values["motor3_pwm"] = int(motor_match.group(3))
        values["motor4_pwm"] = int(motor_match.group(4))

    # Parse fixed BLDC rotation directions reported by the flight controller.
    direction_match = re.search(
        r"dir=(CW|CCW|\+|-)\s+(CW|CCW|\+|-)\s+(CW|CCW|\+|-)\s+(CW|CCW|\+|-)",
        line,
    )
    if direction_match:
        for motor_num in range(1, 5):
            values[f"motor{motor_num}_dir"] = direction_match.group(motor_num)

    # Parse servo and joystick values
    for key in ["servo1", "servo2", "servo3", "servo4", "lx", "ly", "rx", "ry"]:
        match = re.search(rf"{key}=(-?\d+)", line)
        values[key] = int(match.group(1)) if match else 0

    # Parse roll / pitch / yaw floats
    roll_match = re.search(
        r"Roll(?:\s*\[[^\]]*\])?\s*:\s*(-?\d+(?:\.\d+)?)",
        line,
    )
    pitch_match = re.search(
        r"Pitch(?:\s*\[[^\]]*\])?\s*:\s*(-?\d+(?:\.\d+)?)",
        line,
    )
    yaw_match = re.search(
        r"Yaw(?:\s*\[[^\]]*\])?\s*:\s*(-?\d+(?:\.\d+)?)",
        line,
    )
    yaw_rate_match = re.search(
        r"Yaw Rate(?:\s*\[[^\]]*\])?\s*:\s*(-?\d+(?:\.\d+)?)",
        line,
    )

    values["roll_deg"] = float(roll_match.group(1)) if roll_match else 0.0
    values["pitch_deg"] = float(pitch_match.group(1)) if pitch_match else 0.0
    values["yaw_deg"] = float(yaw_match.group(1)) if yaw_match else 0.0
    values["yaw_rate_dps"] = float(yaw_rate_match.group(1)) if yaw_rate_match else 0.0

    values["motor_locked"] = "MOTOR_LOCKED" in line
    values["kill"] = "KILL" in line
    values["failsafe"] = "FAILSAFE" in line
    values["imu_error"] = "IMU_ERROR" in line
    values["pwm_error"] = "PWM_ERROR" in line
    values["bt_error"] = "BT_ERROR" in line
    values["throttle_not_low"] = "THROTTLE_NOT_LOW" in line
    values["servos_dormant"] = "SERVOS_DORMANT" in line
    values["centered"] = "SERVOS_CENTERED" in line or "CENTERED" in line

    return values


def run_self_test():
    """Check the command and telemetry formats without opening the GUI."""
    ack = (
        "ACK motor_pwm=1000 1100 1200 1300 "
        "dir=+ - - + servo1=0 servo2=0 servo3=0 servo4=0 "
        "lx=-20 ly=25 rx=10 ry=-5 "
        "Roll [deg]: 1.25 Pitch [deg]: -2.50 "
        "Yaw [deg]: 12.75 Yaw Rate [deg/s]: 0.50 "
        "MOTOR_LOCKED KILL FAILSAFE PWM_ERROR SERVOS_DORMANT"
    )
    parsed = parse_ack(ack)
    expected = {
        "motor1_pwm": 1000,
        "motor2_pwm": 1100,
        "motor3_pwm": 1200,
        "motor4_pwm": 1300,
        "lx": -20,
        "ly": 25,
        "rx": 10,
        "ry": -5,
        "roll_deg": 1.25,
        "pitch_deg": -2.50,
        "yaw_deg": 12.75,
        "yaw_rate_dps": 0.50,
        "motor1_dir": "+",
        "motor2_dir": "-",
        "motor3_dir": "-",
        "motor4_dir": "+",
    }
    for key, value in expected.items():
        if parsed[key] != value:
            raise AssertionError(f"{key}: expected {value}, got {parsed[key]}")

    if not parsed["motor_locked"] or not parsed["kill"] or not parsed["failsafe"]:
        raise AssertionError("Safety flags were not parsed correctly")
    if not parsed["pwm_error"] or not parsed["servos_dormant"]:
        raise AssertionError("Flight-controller status flags were not parsed correctly")

    safe = safety_payload()
    if safe["cross"] != 1 or safe["r1"] != 0 or safe["ly"] != 0:
        raise AssertionError("Safety command is not motor-disabled")

    print("AmbatuDrone self-test passed: command and ACK formats are compatible.")


class DemoWorker(threading.Thread):
    """Generate dashboard telemetry without a controller or serial hardware."""

    def __init__(self, hz, ui_queue, command_queue):
        super().__init__(daemon=True)
        self.hz = hz
        self.ui_queue = ui_queue
        self.command_queue = command_queue
        self.stop_event = threading.Event()
        self.kill_latched = False

    def send_ui(self, kind, message):
        self.ui_queue.put((kind, message))

    def process_commands(self):
        try:
            while True:
                command = self.command_queue.get_nowait()
                if command == "disconnect":
                    self.stop_event.set()
                    return False
                if command == "kill":
                    self.kill_latched = True
                    self.send_ui("kill_state", True)
                elif command == "reset_kill":
                    self.kill_latched = False
                    self.send_ui("kill_state", False)
        except queue.Empty:
            pass
        return not self.stop_event.is_set()

    def demo_ack(self, elapsed):
        if self.kill_latched:
            motor_values = [1000, 1000, 1000, 1000]
            safety_flags = " MOTOR_LOCKED KILL"
        else:
            # Sweep nearly the full standard ESC command range so Demo Mode
            # verifies both the percentage and the raw 1000-2000 us display.
            base = 1500 + int(450 * math.sin(elapsed * 0.45))
            motor_values = [
                clamp(base + int(35 * math.sin(elapsed + phase)), 1000, 2000)
                for phase in (0.0, 1.2, 2.4, 3.6)
            ]
            safety_flags = ""

        roll = 8.0 * math.sin(elapsed * 0.7)
        pitch = 6.0 * math.cos(elapsed * 0.55)
        yaw = (elapsed * 8.0) % 360.0
        yaw_rate = 8.0
        lx = int(40 * math.sin(elapsed * 0.8))
        ly = int(25 * (1.0 + math.sin(elapsed * 0.9)) / 2.0)
        rx = int(30 * math.sin(elapsed * 0.5))
        ry = int(30 * math.cos(elapsed * 0.5))

        return (
            f"ACK motor_pwm={' '.join(str(value) for value in motor_values)} "
            "dir=+ - - + servo1=0 servo2=0 servo3=0 servo4=0 "
            f"lx={lx} ly={ly} rx={rx} ry={ry} "
            f"Roll [deg]: {roll:.2f} Pitch [deg]: {pitch:.2f} "
            f"Yaw [deg]: {yaw:.2f} Yaw Rate [deg/s]: {yaw_rate:.2f}"
            f"{safety_flags} SERVOS_DORMANT"
        )

    def run(self):
        started = time.monotonic()
        period = 1.0 / max(5.0, min(50.0, float(self.hz)))
        next_update = started
        self.send_ui("connected", "Software demo running — no hardware output")
        self.send_ui(
            "info",
            "Demo mode uses simulated telemetry. Uncheck Demo Mode before a real hardware test.",
        )

        try:
            while not self.stop_event.is_set():
                if not self.process_commands():
                    break
                now = time.monotonic()
                if now >= next_update:
                    self.send_ui("ack", self.demo_ack(now - started))
                    next_update = now + period
                self.stop_event.wait(0.005)
        finally:
            self.send_ui("stopped", "Software demo stopped.")


# -----------------------------
# Background bridge thread
# -----------------------------
class BridgeWorker(threading.Thread):
    """
    Runs the controller -> flight-controller bridge in the background.

    This must be a background thread because the bridge loop repeats constantly.
    If we ran it directly in the GUI thread, the app window would freeze.
    """

    def __init__(
        self,
        port,
        baud,
        hz,
        ui_queue,
        command_queue,
        controller_queue,
        show_raw=False,
    ):
        super().__init__(daemon=True)

        self.port = port
        self.baud = baud
        self.hz = hz
        self.ui_queue = ui_queue
        self.command_queue = command_queue
        self.controller_queue = controller_queue
        self.show_raw = show_raw
        self.stop_event = threading.Event()
        self.ser = None
        self.controller_snapshot = None
        self.serial_rx_buffer = bytearray()
        self.kill_latched = False
        self.telemetry_lost = False
        self.last_ack_time = None
        self.last_r1 = 0
        self.last_cross = 0
        self.last_ly = 0

    def send_ui(self, kind, message):
        self.ui_queue.put((kind, message))

    def send_json(self, payload):
        if self.ser and self.ser.is_open:
            line = json.dumps(payload, separators=(",", ":")) + "\n"
            self.ser.write(line.encode("utf-8"))

    def send_safety_burst(self):
        """Send redundant stop packets so shutdown is safe on a noisy link."""
        for _ in range(3):
            self.send_json(safety_payload())
            self.stop_event.wait(0.02)

    def stop_safely(self, reason):
        self.send_ui("info", reason)
        try:
            self.send_safety_burst()
        except Exception as exc:
            self.send_ui("warning", f"Could not send safety packet: {exc}")
        self.stop_event.set()

    def read_serial_lines(self):
        """Read complete serial lines without blocking on a partial packet."""
        if not self.ser or not self.ser.is_open:
            return []

        waiting = self.ser.in_waiting
        if waiting:
            self.serial_rx_buffer.extend(self.ser.read(waiting))

        if len(self.serial_rx_buffer) > SERIAL_BUFFER_LIMIT:
            self.serial_rx_buffer.clear()
            self.send_ui("warning", "Serial receive buffer overflow; discarded incomplete data.")
            return []

        lines = []
        while b"\n" in self.serial_rx_buffer:
            raw_line, _, remainder = self.serial_rx_buffer.partition(b"\n")
            self.serial_rx_buffer = bytearray(remainder)
            line = raw_line.rstrip(b"\r").decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        return lines

    def handle_serial_line(self, line):
        if line.startswith("ACK "):
            self.last_ack_time = time.monotonic()
            if self.telemetry_lost:
                self.telemetry_lost = False
                self.send_ui("telemetry_recovered", "Telemetry restored; kill remains latched.")
            self.send_ui("ack", line)
            return True

        self.send_ui("info", f"DEVICE: {line}")
        return False

    def process_commands(self):
        """Handle UI commands and return False when the bridge should stop."""
        try:
            while True:
                command = self.command_queue.get_nowait()

                if command == "disconnect":
                    self.stop_safely("Disconnect requested from app.")
                    return False

                if command == "kill":
                    if not self.kill_latched:
                        self.kill_latched = True
                        self.send_ui("kill_state", True)
                    self.send_json(safety_payload())

                elif command == "reset_kill":
                    if self.last_cross:
                        self.send_ui("warning", "Release X before resetting the kill latch.")
                    elif self.last_r1:
                        self.send_ui("warning", "Release R1 before resetting the kill latch.")
                    elif self.last_ly > 5:
                        self.send_ui("warning", "Lower the throttle before resetting the kill latch.")
                    elif self.telemetry_lost:
                        self.send_ui("warning", "Kill cannot be reset while telemetry is missing.")
                    else:
                        self.kill_latched = False
                        self.send_ui("kill_state", False)
                        self.send_ui("info", "Kill latch reset. Motors still require R1 to enable.")

        except queue.Empty:
            pass

        return not self.stop_event.is_set()

    def wait_for_bridge_ack(self):
        """Wait for a real flight-controller ACK over the selected serial port."""
        deadline = time.monotonic() + BRIDGE_STARTUP_TIMEOUT_S
        self.send_ui(
            "info",
            "Waiting for flight-controller telemetry (startup and MPU calibration can take several seconds)...",
        )

        while not self.stop_event.is_set() and time.monotonic() < deadline:
            if not self.process_commands():
                return False

            for line in self.read_serial_lines():
                if self.handle_serial_line(line):
                    return True

            self.stop_event.wait(0.02)

        if self.stop_event.is_set():
            return False

        raise RuntimeError(
            "The serial port opened, but no AmbatuDrone ACK arrived within 12 seconds. "
            "Check the selected port, set 115200 baud, and confirm the Teensy "
            "flight controller or legacy ESP32 firmware is running."
        )

    def update_controller_snapshot(self):
        """Use the newest controller state captured by Tk's main thread."""
        try:
            while True:
                self.controller_snapshot = self.controller_queue.get_nowait()
        except queue.Empty:
            pass

        return self.controller_snapshot

    def controller_payload(self):
        snapshot = self.update_controller_snapshot()
        if snapshot is None:
            raise RuntimeError("Controller input was not initialized.")

        if time.monotonic() - snapshot["captured_at"] > 0.5:
            if not self.kill_latched:
                self.kill_latched = True
                self.send_ui("kill_state", True)
                self.send_ui("warning", "Controller input timed out; kill is latched.")
            return safety_payload()

        axes = snapshot["axes"]
        buttons = snapshot["buttons"]

        # SDL axes: left X/Y and right X/Y. Upward stick motion is negative
        # in pygame, so Y axes are inverted before sending them to the controller.
        raw_lx = get_snapshot_value(axes, AXIS_LX, 0.0)
        raw_ly = get_snapshot_value(axes, AXIS_LY, 0.0)
        raw_rx = get_snapshot_value(axes, AXIS_RX, 0.0)
        raw_ry = get_snapshot_value(axes, AXIS_RY, 0.0)

        cross = int(get_snapshot_value(buttons, BUTTON_CROSS, 0))
        self.last_cross = cross
        self.last_r1 = int(get_snapshot_value(buttons, BUTTON_R1, 0))
        self.last_ly = normalize_axis(-raw_ly)

        # X is a latched kill. Releasing X does not restart the motors.
        if cross and not self.kill_latched:
            self.kill_latched = True
            self.send_ui("kill_state", True)
            self.send_ui("warning", "Controller X pressed; kill is latched.")

        if self.kill_latched or self.telemetry_lost:
            return safety_payload()

        return {
            "lx": normalize_axis(raw_lx),
            "ly": self.last_ly,
            "rx": normalize_axis(raw_rx),
            "ry": normalize_axis(-raw_ry),
            "cross": cross,
            "circle": int(get_snapshot_value(buttons, BUTTON_CIRCLE, 0)),
            "square": int(get_snapshot_value(buttons, BUTTON_SQUARE, 0)),
            "triangle": int(get_snapshot_value(buttons, BUTTON_TRIANGLE, 0)),
            "l1": int(get_snapshot_value(buttons, BUTTON_L1, 0)),
            "r1": self.last_r1,
        }

    def run(self):
        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0,
                write_timeout=0.25,
            )

            if not self.wait_for_bridge_ack():
                return

            self.send_ui("connected", f"Bridge verified on {self.port} @ {self.baud}")
            self.send_ui(
                "info",
                "Controls: R1 = enable, left Y = throttle, left X = yaw, "
                "right stick = roll/pitch, X = latched kill.",
            )

            period = 1.0 / max(1.0, float(self.hz))
            last_raw_print = 0.0
            next_send_time = time.monotonic()

            while not self.stop_event.is_set():
                if not self.process_commands():
                    break

                now = time.monotonic()
                if now >= next_send_time:
                    self.send_json(self.controller_payload())
                    next_send_time += period
                    if next_send_time < now:
                        next_send_time = now + period

                for line in self.read_serial_lines():
                    self.handle_serial_line(line)

                if (
                    self.last_ack_time is not None
                    and not self.telemetry_lost
                    and time.monotonic() - self.last_ack_time > TELEMETRY_TIMEOUT_S
                ):
                    self.telemetry_lost = True
                    self.kill_latched = True
                    self.send_ui("kill_state", True)
                    self.send_ui(
                        "telemetry_stale",
                        "Telemetry timed out. Safety commands are being sent; kill is latched.",
                    )

                if self.show_raw and time.time() - last_raw_print > 0.5:
                    snapshot = self.update_controller_snapshot()
                    if snapshot:
                        axes = [round(value, 3) for value in snapshot["axes"]]
                        buttons = snapshot["buttons"]
                        self.send_ui("info", f"RAW axes={axes} buttons={buttons}")
                    last_raw_print = time.time()

                self.stop_event.wait(0.005)

        except Exception as exc:
            self.send_ui("error", str(exc))

        finally:
            try:
                self.send_safety_burst()
            except Exception:
                pass

            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

            self.send_ui("stopped", "Bridge stopped. Safety command sent.")


# -----------------------------
# Main app class
# -----------------------------
class DroneApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title(f"{APP_NAME} Dashboard {APP_VERSION}")
        self.geometry("1100x820")
        self.minsize(1000, 700)

        self.port_map = {}
        self.worker = None
        self.controller = None
        self.controller_queue = None
        self.controller_poll_id = None
        self.ui_queue = queue.Queue()
        self.command_queue = queue.Queue()
        self.kill_latched = False
        self.closing = False
        self.close_deadline = None

        # Tkinter variables
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.hz_var = tk.StringVar(value=str(DEFAULT_HZ))
        self.show_raw_var = tk.BooleanVar(value=False)
        self.demo_mode_var = tk.BooleanVar(value=True)

        self.connection_var = tk.StringVar(
            value="Demo Mode selected — no hardware commands will be sent"
        )

        # Flight telemetry display variables
        self.roll_var = tk.StringVar(value="0.00°")
        self.pitch_var = tk.StringVar(value="0.00°")
        self.yaw_var = tk.StringVar(value="0.00°")
        self.yaw_rate_var = tk.StringVar(value="0.00°/s")

        # Raw and displayed attitude values
        self.last_raw_yaw = 0.0
        self.yaw_zero_offset = 0.0

        self.display_roll = 0.0
        self.display_pitch = 0.0
        self.display_yaw = 0.0

        self.motor1_speed_var = tk.StringVar(value="0%")
        self.motor2_speed_var = tk.StringVar(value="0%")
        self.motor3_speed_var = tk.StringVar(value="0%")
        self.motor4_speed_var = tk.StringVar(value="0%")

        self.motor1_pwm_var = tk.StringVar(value="PWM: 1000 µs")
        self.motor2_pwm_var = tk.StringVar(value="PWM: 1000 µs")
        self.motor3_pwm_var = tk.StringVar(value="PWM: 1000 µs")
        self.motor4_pwm_var = tk.StringVar(value="PWM: 1000 µs")

        self.motor1_direction_var = tk.StringVar(value="0 Stopped")
        self.motor2_direction_var = tk.StringVar(value="0 Stopped")
        self.motor3_direction_var = tk.StringVar(value="0 Stopped")
        self.motor4_direction_var = tk.StringVar(value="0 Stopped")

        self.servo1_var = tk.StringVar(value="0°")
        self.servo2_var = tk.StringVar(value="0°")
        self.servo3_var = tk.StringVar(value="0°")
        self.servo4_var = tk.StringVar(value="0°")

        self.lx_var = tk.StringVar(value="0")
        self.ly_var = tk.StringVar(value="0")
        self.rx_var = tk.StringVar(value="0")
        self.ry_var = tk.StringVar(value="0")

        self.motor_lock_var = tk.StringVar(value="Motor Locked")
        self.kill_var = tk.StringVar(value="Kill Switch Off")
        self.failsafe_var = tk.StringVar(value="Failsafe Unknown")
        self.imu_var = tk.StringVar(value="IMU Unknown")
        self.throttle_check_var = tk.StringVar(value="Throttle Check Unknown")
        self.centered_var = tk.StringVar(value="Servos Dormant")

        self.new_ack_var = tk.StringVar(value="Waiting for data...")

        self.build_ui()
        self.refresh_ports()
        self.update_demo_controls()
        self.after(100, self.process_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def wrap_angle(self, angle):
        """Keep yaw between -180 and +180 degrees."""
        return (angle + 180.0) % 360.0 - 180.0

    def zero_yaw(self):
        """Treat the drone's current direction as zero degrees."""
        self.yaw_zero_offset = self.last_raw_yaw
        self.yaw_var.set("0.00°")
        self.log("Yaw reference reset to 0°.")

    # -----------------------------
    # Building the UI
    # -----------------------------
    def build_ui(self):
        root = ttk.Frame(self, padding=15)
        root.pack(fill="both", expand=True)

        title = ttk.Label(root, text=APP_NAME, font=("Times New Roman", 22, "bold"))
        title.pack(anchor="w")

        subtitle = ttk.Label(
            root,
            text="PS4 controller -> AmbatuDrone app -> Teensy/ESP32 flight controller -> ESCs",
        )
        subtitle.pack(anchor="w", pady=(0, 15))

        # Connection frame
        connection_box = ttk.LabelFrame(root, text="Connection", padding=10)
        connection_box.pack(fill="x")
        connection_box.columnconfigure(1, weight=1)

        ttk.Label(connection_box, text="Flight Controller / Ground Radio Serial Port:").grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.port_combo = ttk.Combobox(connection_box, textvariable=self.port_var, width=45)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)

        self.refresh_button = ttk.Button(
            connection_box,
            text="Refresh ports",
            command=self.refresh_ports,
        )
        self.refresh_button.grid(row=0, column=2, padx=5)

        ttk.Label(connection_box, text="Baud:").grid(row=0, column=3, sticky="e", padx=(12, 4))
        ttk.Entry(connection_box, textvariable=self.baud_var, width=9).grid(row=0, column=4, sticky="w")

        ttk.Label(connection_box, text="Hz:").grid(row=0, column=5, sticky="e", padx=(12, 4))
        ttk.Entry(connection_box, textvariable=self.hz_var, width=7).grid(row=0, column=6, sticky="w")

        self.connect_button = ttk.Button(connection_box, text="Connect", command=self.connect)
        self.connect_button.grid(row=1, column=0, pady=10, sticky="ew")

        self.disconnect_button = ttk.Button(connection_box, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_button.grid(row=1, column=1, pady=10, sticky="w")

        self.kill_button = ttk.Button(connection_box, text="KILL / STOP", command=self.kill_bridge, state="disabled")
        self.kill_button.grid(row=1, column=2, pady=10, sticky="ew")

        raw_check = ttk.Checkbutton(connection_box, text="Raw controller debug", variable=self.show_raw_var)
        raw_check.grid(row=2, column=0, sticky="w")

        self.demo_check = ttk.Checkbutton(
            connection_box,
            text="Demo Mode (no controller or board)",
            variable=self.demo_mode_var,
            command=self.update_demo_controls,
        )
        self.demo_check.grid(row=2, column=1, columnspan=2, sticky="w", padx=8)

        ttk.Label(connection_box, textvariable=self.connection_var).grid(
            row=3, column=0, columnspan=7, sticky="w", pady=(5, 0)
        )

        # Motor status frame
        motor_box = ttk.LabelFrame(root, text="Motor Status", padding=10)
        motor_box.pack(fill="x", pady=(15, 10))

        for i in range(4):
            motor_box.columnconfigure(i, weight=1)

        self.motor1_bar = ttk.Progressbar(motor_box, maximum=100)
        self.motor2_bar = ttk.Progressbar(motor_box, maximum=100)
        self.motor3_bar = ttk.Progressbar(motor_box, maximum=100)
        self.motor4_bar = ttk.Progressbar(motor_box, maximum=100)

        self.make_metric(motor_box, "Motor 1", self.motor1_speed_var, 0, 0)
        self.motor1_bar.grid(row=1, column=0, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor1_direction_var, font=("Times New Roman", 12)).grid(row=2, column=0, pady=(5, 0))
        ttk.Label(motor_box, textvariable=self.motor1_pwm_var, font=("Times New Roman", 11, "bold")).grid(row=3, column=0, pady=(2, 0))

        self.make_metric(motor_box, "Motor 2", self.motor2_speed_var, 0, 1)
        self.motor2_bar.grid(row=1, column=1, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor2_direction_var, font=("Times New Roman", 12)).grid(row=2, column=1, pady=(5, 0))
        ttk.Label(motor_box, textvariable=self.motor2_pwm_var, font=("Times New Roman", 11, "bold")).grid(row=3, column=1, pady=(2, 0))

        self.make_metric(motor_box, "Motor 3", self.motor3_speed_var, 0, 2)
        self.motor3_bar.grid(row=1, column=2, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor3_direction_var, font=("Times New Roman", 12)).grid(row=2, column=2, pady=(5, 0))
        ttk.Label(motor_box, textvariable=self.motor3_pwm_var, font=("Times New Roman", 11, "bold")).grid(row=3, column=2, pady=(2, 0))

        self.make_metric(motor_box, "Motor 4", self.motor4_speed_var, 0, 3)
        self.motor4_bar.grid(row=1, column=3, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor4_direction_var, font=("Times New Roman", 12)).grid(row=2, column=3, pady=(5, 0))
        ttk.Label(motor_box, textvariable=self.motor4_pwm_var, font=("Times New Roman", 11, "bold")).grid(row=3, column=3, pady=(2, 0))

        # Servo status frame
        servo_box = ttk.LabelFrame(root, text="Servo Status", padding=10)
        servo_box.pack(fill="x", pady=(0, 15))

        for i in range(4):
            servo_box.columnconfigure(i, weight=1)

        self.servo1_bar = ttk.Progressbar(servo_box, maximum=180)
        self.servo2_bar = ttk.Progressbar(servo_box, maximum=180)
        self.servo3_bar = ttk.Progressbar(servo_box, maximum=180)
        self.servo4_bar = ttk.Progressbar(servo_box, maximum=180)

        self.make_metric(servo_box, "Servo 1", self.servo1_var, 0, 0)
        self.servo1_bar.grid(row=1, column=0, sticky="ew", padx=10)

        self.make_metric(servo_box, "Servo 2", self.servo2_var, 0, 1)
        self.servo2_bar.grid(row=1, column=1, sticky="ew", padx=10)

        self.make_metric(servo_box, "Servo 3", self.servo3_var, 0, 2)
        self.servo3_bar.grid(row=1, column=2, sticky="ew", padx=10)

        self.make_metric(servo_box, "Servo 4", self.servo4_var, 0, 3)
        self.servo4_bar.grid(row=1, column=3, sticky="ew", padx=10)

        telemetry_box = ttk.LabelFrame(root, text="Flight Telemetry", padding=10)
        telemetry_box.pack(fill="x", pady=(0, 15))

        for i in range(5):
            telemetry_box.columnconfigure(i, weight=1)

        self.make_metric(telemetry_box, "Roll", self.roll_var, 0, 0)
        self.make_metric(telemetry_box, "Pitch", self.pitch_var, 0, 1)
        self.make_metric(telemetry_box, "Relative Yaw", self.yaw_var, 0, 2)
        self.make_metric(telemetry_box, "Yaw Rate", self.yaw_rate_var, 0, 3)

        zero_yaw_button = ttk.Button(
            telemetry_box,
            text="Zero Yaw",
            command=self.zero_yaw
        )
        zero_yaw_button.grid(row=0, column=4, padx=10, pady=5)

        # Joystick axes frame
        axes_box = ttk.LabelFrame(root, text="Joystick Axes", padding=10)
        axes_box.pack(fill="x")

        for i in range(4):
            axes_box.columnconfigure(i, weight=1)

        self.make_metric(axes_box, "LX", self.lx_var, 0, 0)
        self.make_metric(axes_box, "LY", self.ly_var, 0, 1)
        self.make_metric(axes_box, "RX", self.rx_var, 0, 2)
        self.make_metric(axes_box, "RY", self.ry_var, 0, 3)

        # Safety status frame
        safety_box = ttk.LabelFrame(root, text="Safety Status", padding=10)
        safety_box.pack(fill="x", pady=15)

        for i in range(6):
            safety_box.columnconfigure(i, weight=1)

        self.make_metric(safety_box, "Motor Lock", self.motor_lock_var, 0, 0)
        self.make_metric(safety_box, "Kill Switch", self.kill_var, 0, 1)
        self.make_metric(safety_box, "Failsafe", self.failsafe_var, 0, 2)
        self.make_metric(safety_box, "IMU", self.imu_var, 0, 3)
        self.make_metric(safety_box, "Arm Check", self.throttle_check_var, 0, 4)
        self.make_metric(safety_box, "Servo State", self.centered_var, 0, 5)

        # Raw ACK frame
        ack_box = ttk.LabelFrame(root, text="ACK Data", padding=10)
        ack_box.pack(fill="both", expand=True)

        ack_label = ttk.Label(ack_box, textvariable=self.new_ack_var, wraplength=900)
        ack_label.pack(anchor="w")

        # Log frame
        log_box = ttk.LabelFrame(root, text="App Log", padding=10)
        log_box.pack(fill="both", expand=True, pady=(10, 0))

        self.log_text = tk.Text(log_box, height=7, wrap="word", state="disabled")
        self.log_text.pack(fill="both", expand=True)

    # -----------------------------
    # UI helper
    # -----------------------------
    def make_metric(self, parent, label, variable, row, column):
        frame = ttk.Frame(parent)
        frame.grid(row=row, column=column, sticky="nsew", padx=10, pady=5)

        title = ttk.Label(frame, text=label, font=("Times New Roman", 11, "bold"))
        title.pack()

        value = ttk.Label(frame, textvariable=variable, font=("Times New Roman", 16))
        value.pack()

    # -----------------------------
    # Log helper
    # -----------------------------
    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.configure(state="normal")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    # -----------------------------
    # Refresh COM ports
    # -----------------------------
    def refresh_ports(self):
        previously_selected = self.selected_port() if self.port_map else ""
        ports = list(list_ports.comports())
        self.port_map.clear()
        display_values = []

        for port in ports:
            display_text = f"{port.device} - {port.description}"
            self.port_map[display_text] = port.device
            display_values.append(display_text)

        self.port_combo["values"] = display_values

        if display_values:
            restored = next(
                (
                    display
                    for display, device in self.port_map.items()
                    if device == previously_selected
                ),
                display_values[0],
            )
            self.port_var.set(restored)
            self.log(f"Found {len(display_values)} serial port(s).")
        else:
            self.port_var.set("No Ports Found")
            self.log("No serial ports found.")

    def update_demo_controls(self, update_status=True):
        """Make it obvious whether Connect uses real hardware or simulation."""
        if self.demo_mode_var.get():
            self.port_combo.configure(state="disabled")
            self.refresh_button.configure(state="disabled")
            self.connect_button.configure(text="Start Demo")
            if update_status and (not self.worker or not self.worker.is_alive()):
                self.connection_var.set(
                    "Demo Mode selected — no hardware commands will be sent"
                )
        else:
            self.port_combo.configure(state="normal")
            self.refresh_button.configure(state="normal")
            self.connect_button.configure(text="Connect")
            if update_status and (not self.worker or not self.worker.is_alive()):
                self.connection_var.set("Disconnected")

    def selected_port(self):
        selected = self.port_var.get().strip()
        return self.port_map.get(selected, selected)

    # -----------------------------
    # Controller handling
    # -----------------------------
    def start_controller(self):
        """
        Initialize SDL and the controller on Tk's main thread.

        macOS requires SDL video/input initialization and event pumping on the
        main thread. Serial I/O remains in BridgeWorker.
        """
        self.stop_controller()
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            pygame.quit()
            raise RuntimeError(
                "No controller detected. Connect the PS4 controller and try again."
            )

        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.controller_queue = queue.Queue(maxsize=1)
        self.capture_controller_snapshot()

        self.log(f"Controller connected: {self.controller.get_name()}")
        self.log(
            f"Axes: {self.controller.get_numaxes()} | "
            f"Buttons: {self.controller.get_numbuttons()} | "
            f"Hats: {self.controller.get_numhats()}"
        )

    def capture_controller_snapshot(self):
        """Capture one controller state on Tk's main thread."""
        if not self.controller or not self.controller.get_init():
            raise RuntimeError("The PS4 controller disconnected.")

        pygame.event.pump()
        snapshot = {
            "captured_at": time.monotonic(),
            "axes": [
                get_axis_safe(self.controller, index)
                for index in range(self.controller.get_numaxes())
            ],
            "buttons": [
                get_button_safe(self.controller, index)
                for index in range(self.controller.get_numbuttons())
            ],
        }

        try:
            while True:
                self.controller_queue.get_nowait()
        except queue.Empty:
            pass

        self.controller_queue.put_nowait(snapshot)

    def poll_controller(self):
        """Keep controller input current without invoking SDL from a worker."""
        self.controller_poll_id = None
        if not self.worker or not self.worker.is_alive() or not self.controller:
            return

        try:
            self.capture_controller_snapshot()
        except Exception as exc:
            self.connection_var.set("Controller lost — safety stop active")
            self.log(f"ERROR: {exc}")
            self.command_queue.put("kill")
            self.command_queue.put("disconnect")
            self.stop_controller()
            return

        self.controller_poll_id = self.after(10, self.poll_controller)

    def stop_controller(self):
        """Release Pygame/SDL resources on Tk's main thread."""
        if self.controller_poll_id is not None:
            try:
                self.after_cancel(self.controller_poll_id)
            except Exception:
                pass
            self.controller_poll_id = None

        try:
            if self.controller:
                self.controller.quit()
            pygame.quit()
        except Exception:
            pass

        self.controller = None
        self.controller_queue = None

    # -----------------------------
    # Connect button behavior
    # -----------------------------
    def connect(self):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Already connected", "The bridge is already running.")
            return

        try:
            hz = float(self.hz_var.get())
            if not 5.0 <= hz <= 50.0:
                raise ValueError
        except ValueError:
            messagebox.showerror(
                "Bad settings",
                "Command rate must be between 5 and 50 Hz.",
            )
            return

        self.ui_queue = queue.Queue()
        self.command_queue = queue.Queue()
        self.kill_latched = False

        if self.demo_mode_var.get():
            self.worker = DemoWorker(
                hz=hz,
                ui_queue=self.ui_queue,
                command_queue=self.command_queue,
            )
            connection_target = "Software Demo"
            log_message = f"Starting software demo at {hz} Hz."
        else:
            port = self.selected_port()
            if not port or port == "No Ports Found":
                messagebox.showerror(
                    "No port selected",
                    "Connect the Teensy, ground Feather, or legacy ESP32, then select its serial port.",
                )
                return

            try:
                baud = int(self.baud_var.get())
                if baud <= 0:
                    raise ValueError
            except ValueError:
                messagebox.showerror("Bad settings", "Baud must be a positive integer.")
                return

            try:
                self.start_controller()
            except Exception as exc:
                self.stop_controller()
                messagebox.showerror("Controller error", str(exc))
                return

            self.worker = BridgeWorker(
                port=port,
                baud=baud,
                hz=hz,
                ui_queue=self.ui_queue,
                command_queue=self.command_queue,
                controller_queue=self.controller_queue,
                show_raw=self.show_raw_var.get(),
            )
            connection_target = port
            log_message = f"Starting bridge on {port} @ {baud}, {hz} Hz."

        self.worker.start()
        if not self.demo_mode_var.get():
            self.controller_poll_id = self.after(10, self.poll_controller)

        self.connection_var.set(f"Connecting to {connection_target}...")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.kill_button.config(text="KILL / STOP", state="disabled")
        self.demo_check.config(state="disabled")
        self.log(log_message)

    # -----------------------------
    # Disconnect button behavior
    # -----------------------------
    def disconnect(self):
        if self.worker and self.worker.is_alive():
            self.command_queue.put("disconnect")
            self.connection_var.set("Disconnecting safely...")
            self.disconnect_button.config(state="disabled")
            self.kill_button.config(state="disabled")
            self.log("Disconnect requested.")

    # -----------------------------
    # Kill button behavior
    # -----------------------------
    def kill_bridge(self):
        if self.worker and self.worker.is_alive():
            if self.kill_latched:
                self.command_queue.put("reset_kill")
                self.log("Kill reset requested.")
            else:
                self.command_queue.put("kill")
                self.log("KILL / STOP pressed. Safety command latched.")

    def set_kill_state(self, latched):
        self.kill_latched = bool(latched)
        if self.kill_latched:
            self.kill_button.config(text="RESET KILL")
            self.kill_var.set("Kill Latched")
        else:
            self.kill_button.config(text="KILL / STOP")
            self.kill_var.set("Kill Switch Off")

    # -----------------------------
    # Update dashboard from flight-controller ACK
    # -----------------------------
    def format_motor_direction(self, raw_direction):
        """Convert Arduino direction symbols into dashboard text."""
        if raw_direction in ["+", "CW"]:
            return "CW"
        if raw_direction in ["-", "CCW"]:
            return "CCW"
        if raw_direction == "N/A":
            return "Direction not reported"
        return "Stopped"

    def update_motor_display(self, motor_num, pwm, direction):
        """Update one motor's percent bar, direction text, and raw PWM pulse."""
        motor_percent = int((pwm - 1000) / (2000 - 1000) * 100)
        motor_percent = max(0, min(100, motor_percent))

        if motor_num == 1:
            self.motor1_speed_var.set(f"{motor_percent}%")
            self.motor1_bar["value"] = motor_percent
            self.motor1_direction_var.set(self.format_motor_direction(direction))
            self.motor1_pwm_var.set(f"PWM: {pwm} µs")
        elif motor_num == 2:
            self.motor2_speed_var.set(f"{motor_percent}%")
            self.motor2_bar["value"] = motor_percent
            self.motor2_direction_var.set(self.format_motor_direction(direction))
            self.motor2_pwm_var.set(f"PWM: {pwm} µs")
        elif motor_num == 3:
            self.motor3_speed_var.set(f"{motor_percent}%")
            self.motor3_bar["value"] = motor_percent
            self.motor3_direction_var.set(self.format_motor_direction(direction))
            self.motor3_pwm_var.set(f"PWM: {pwm} µs")
        elif motor_num == 4:
            self.motor4_speed_var.set(f"{motor_percent}%")
            self.motor4_bar["value"] = motor_percent
            self.motor4_direction_var.set(self.format_motor_direction(direction))
            self.motor4_pwm_var.set(f"PWM: {pwm} µs")

    # -----------------------------
    # Update dashboard from flight-controller ACK
    # -----------------------------
    def update_dashboard(self, ack_line):
        data = parse_ack(ack_line)

        # Update roll, pitch, relative yaw angle, and live yaw rate
        self.last_raw_yaw = data["yaw_deg"]

        relative_yaw = self.wrap_angle(
            self.last_raw_yaw - self.yaw_zero_offset
        )

        self.roll_var.set(f"{data['roll_deg']:.2f}°")
        self.pitch_var.set(f"{data['pitch_deg']:.2f}°")
        self.yaw_var.set(f"{relative_yaw:.2f}°")
        yaw_rate = 0.0 if abs(data["yaw_rate_dps"]) < 0.005 else data["yaw_rate_dps"]
        self.yaw_rate_var.set(f"{yaw_rate:+.2f}°/s")

        for motor_num in range(1, 5):
            self.update_motor_display(
                motor_num=motor_num,
                pwm=data[f"motor{motor_num}_pwm"],
                direction=data[f"motor{motor_num}_dir"],
            )

        self.servo1_var.set(f"{data['servo1']}°")
        self.servo1_bar["value"] = max(0, min(180, data["servo1"]))

        self.servo2_var.set(f"{data['servo2']}°")
        self.servo2_bar["value"] = max(0, min(180, data["servo2"]))

        self.servo3_var.set(f"{data['servo3']}°")
        self.servo3_bar["value"] = max(0, min(180, data["servo3"]))

        self.servo4_var.set(f"{data['servo4']}°")
        self.servo4_bar["value"] = max(0, min(180, data["servo4"]))

        self.lx_var.set(str(data["lx"]))
        self.ly_var.set(str(data["ly"]))
        self.rx_var.set(str(data["rx"]))
        self.ry_var.set(str(data["ry"]))

        if data["pwm_error"]:
            self.motor_lock_var.set("PWM Output Error")
        else:
            self.motor_lock_var.set(
                "Motor Locked" if data["motor_locked"] else "Motor Enabled"
            )
        if data["kill"]:
            self.kill_var.set("Kill Active")
        elif self.kill_latched:
            self.kill_var.set("Kill Latched")
        else:
            self.kill_var.set("Kill Switch Off")

        self.failsafe_var.set("Failsafe Active" if data["failsafe"] else "Link Healthy")
        self.imu_var.set("IMU Error" if data["imu_error"] else "IMU Ready")
        self.throttle_check_var.set(
            "Lower Throttle" if data["throttle_not_low"] else "Throttle OK"
        )
        if data["servos_dormant"]:
            self.centered_var.set("Servos Dormant")
        elif data["centered"]:
            self.centered_var.set("Servos Centered")
        else:
            self.centered_var.set("Servo State Unknown")

        self.new_ack_var.set(ack_line)

    # -----------------------------
    # Handle messages from background thread
    # -----------------------------
    def process_ui_queue(self):
        try:
            while True:
                kind, message = self.ui_queue.get_nowait()

                if kind == "ack":
                    self.update_dashboard(message)
                elif kind == "connected":
                    self.connection_var.set(message)
                    self.kill_button.config(state="normal")
                    self.log(message)
                elif kind == "stopped":
                    self.stop_controller()
                    self.connection_var.set("Disconnected")
                    self.connect_button.config(state="normal")
                    self.disconnect_button.config(state="disabled")
                    self.kill_button.config(state="disabled")
                    self.demo_check.config(state="normal")
                    self.update_demo_controls(update_status=False)
                    self.log(message)
                elif kind == "error":
                    self.connection_var.set("Error / Disconnected")
                    self.connect_button.config(state="normal")
                    self.disconnect_button.config(state="disabled")
                    self.kill_button.config(state="disabled")
                    self.demo_check.config(state="normal")
                    self.update_demo_controls(update_status=False)
                    self.log(f"ERROR: {message}")
                    if not self.closing:
                        messagebox.showerror("Bridge error", message)
                elif kind == "kill_state":
                    self.set_kill_state(message)
                elif kind == "telemetry_stale":
                    self.connection_var.set("Telemetry lost — safety stop active")
                    self.log(f"WARNING: {message}")
                elif kind == "telemetry_recovered":
                    self.connection_var.set("Telemetry restored — kill remains latched")
                    self.log(message)
                elif kind == "warning":
                    self.log(f"WARNING: {message}")
                else:
                    self.log(message)

        except queue.Empty:
            pass

        self.after(100, self.process_ui_queue)

    # -----------------------------
    # Window close behavior
    # -----------------------------
    def on_close(self):
        if self.closing:
            return

        self.closing = True
        if self.worker and self.worker.is_alive():
            self.command_queue.put("disconnect")
            self.connection_var.set("Closing safely...")
            self.connect_button.config(state="disabled")
            self.disconnect_button.config(state="disabled")
            self.kill_button.config(state="disabled")
            self.close_deadline = time.monotonic() + 1.5
            self.after(50, self.finish_close)
        else:
            self.stop_controller()
            self.destroy()

    def finish_close(self):
        if (
            self.worker
            and self.worker.is_alive()
            and time.monotonic() < self.close_deadline
        ):
            self.after(50, self.finish_close)
            return

        self.stop_controller()
        self.destroy()


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        run_self_test()
    else:
        app = DroneApp()
        app.mainloop()
