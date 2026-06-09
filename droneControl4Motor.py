# Drone app V1.0 - Hardware-only version
# PS4 controller -> Python app bridge -> Arduino -> dashboard display

import json
import queue
import re
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import pygame
import serial
from serial.tools import list_ports


DEFAULT_BAUD = 115200
DEFAULT_HZ = 25.0


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
    """Take one Arduino status line and convert it into a Python dictionary.

    Expected 4-motor ACK format:
        ACK motor1_pwm=0 motor1_dir=0 motor2_pwm=0 motor2_dir=0 motor3_pwm=0 motor3_dir=0 motor4_pwm=0 motor4_dir=0 servo1=90 servo2=90 servo3=90 servo4=90 lx=0 ly=0 rx=0 ry=0 MOTOR_LOCKED

    Backward compatibility:
        If the Arduino still prints motor_pwm and motor_dir, those values are used for motor 1.
    """
    values = {}

    # Motor PWM values. New format is motor1_pwm ... motor4_pwm.
    # Old format motor_pwm still maps to motor1_pwm so older Arduino code does not fully break.
    old_motor_pwm = re.search(r"motor_pwm=(-?\d+)", line)
    old_motor_dir = re.search(r"motor_dir=([+\-0]|CW|CCW)", line)

    for motor_num in range(1, 5):
        pwm_key = f"motor{motor_num}_pwm"
        dir_key = f"motor{motor_num}_dir"

        pwm_match = re.search(rf"{pwm_key}=(-?\d+)", line)
        dir_match = re.search(rf"{dir_key}=([+\-0]|CW|CCW)", line)

        # Backward compatibility for the original single-motor names.
        if motor_num == 1 and not pwm_match:
            pwm_match = old_motor_pwm
        if motor_num == 1 and not dir_match:
            dir_match = old_motor_dir

        values[pwm_key] = int(pwm_match.group(1)) if pwm_match else 0
        values[dir_key] = dir_match.group(1) if dir_match else "0"

    # Servo and joystick values.
    for key in ["servo1", "servo2", "servo3", "servo4", "lx", "ly", "rx", "ry"]:
        match = re.search(rf"{key}=(-?\d+)", line)
        values[key] = int(match.group(1)) if match else 0

    values["motor_locked"] = "MOTOR_LOCKED" in line
    values["kill"] = "KILL" in line
    values["centered"] = "SERVOS_CENTERED" in line or "CENTERED" in line

    return values


# -----------------------------
# Background bridge thread
# -----------------------------
class BridgeWorker(threading.Thread):
    """
    Runs the controller -> Arduino bridge in the background.

    This must be a background thread because the bridge loop repeats constantly.
    If we ran it directly in the GUI thread, the app window would freeze.
    """

    def __init__(self, port, baud, hz, ui_queue, command_queue, show_raw=False):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.hz = hz
        self.ui_queue = ui_queue
        self.command_queue = command_queue
        self.show_raw = show_raw
        self.stop_event = threading.Event()
        self.ser = None
        self.joystick = None

    def send_ui(self, kind, message):
        self.ui_queue.put((kind, message))

    def send_json(self, payload):
        if self.ser and self.ser.is_open:
            line = json.dumps(payload, separators=(",", ":")) + "\n"
            self.ser.write(line.encode("utf-8"))

    def stop_safely(self, reason):
        self.send_ui("info", reason)
        try:
            self.send_json(safety_payload())
            time.sleep(0.05)
        except Exception as exc:
            self.send_ui("warning", f"Could not send safety packet: {exc}")
        self.stop_event.set()

    def run(self):
        try:
            pygame.init()
            pygame.joystick.init()

            if pygame.joystick.get_count() == 0:
                raise RuntimeError("No controller detected. Connect the PS4 controller and try again.")

            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()

            self.send_ui("info", f"Controller connected: {self.joystick.get_name()}")
            self.send_ui(
                "info",
                f"Axes: {self.joystick.get_numaxes()} | Buttons: {self.joystick.get_numbuttons()} | Hats: {self.joystick.get_numhats()}",
            )

            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2.0)  # Arduino usually resets when serial opens.
            self.ser.reset_input_buffer()

            self.send_ui("connected", f"Connected to Arduino on {self.port} @ {self.baud}")
            self.send_ui(
                "info",
                "Controls: hold R1 for motor enable, left stick Y = throttle, right stick = servos, X = kill, L1 = center servos",
            )

            period = 1.0 / max(1.0, float(self.hz))
            last_raw_print = 0.0

            while not self.stop_event.is_set():
                # Check for app button commands.
                try:
                    while True:
                        command = self.command_queue.get_nowait()

                        if command == "kill":
                            self.stop_safely("KILL requested from app.")
                        elif command == "disconnect":
                            self.stop_safely("Disconnect requested from app.")

                except queue.Empty:
                    pass

                if self.stop_event.is_set():
                    break

                pygame.event.pump()

                # Common DualShock 4 mappings in pygame:
                # axis 0 = left stick X
                # axis 1 = left stick Y, negative when pushed upward
                # axis 2 = right stick X on many systems
                # axis 3 or 5 = right stick Y depending on OS/driver
                raw_lx = get_axis_safe(self.joystick, 0)
                raw_ly = get_axis_safe(self.joystick, 1)
                raw_rx = get_axis_safe(self.joystick, 2)
                raw_ry = get_axis_safe(self.joystick, 3, get_axis_safe(self.joystick, 5))

                payload = {
                    "lx": normalize_axis(raw_lx),
                    "ly": normalize_axis(-raw_ly),  # push up = positive throttle
                    "rx": normalize_axis(raw_rx),
                    "ry": normalize_axis(-raw_ry),
                    "cross": get_button_safe(self.joystick, 0),
                    "circle": get_button_safe(self.joystick, 1),
                    "square": get_button_safe(self.joystick, 2),
                    "triangle": get_button_safe(self.joystick, 3),
                    "l1": get_button_safe(self.joystick, 4),
                    "r1": get_button_safe(self.joystick, 10),
                }

                self.send_json(payload)

                while self.ser and self.ser.in_waiting:
                    ack = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if ack:
                        self.send_ui("ack", ack)

                if self.show_raw and time.time() - last_raw_print > 0.5:
                    axes = [round(get_axis_safe(self.joystick, i), 3) for i in range(self.joystick.get_numaxes())]
                    buttons = [get_button_safe(self.joystick, i) for i in range(self.joystick.get_numbuttons())]
                    self.send_ui("info", f"RAW axes={axes} buttons={buttons}")
                    last_raw_print = time.time()

                time.sleep(period)

        except Exception as exc:
            self.send_ui("error", str(exc))

        finally:
            try:
                self.send_json(safety_payload())
                time.sleep(0.05)
            except Exception:
                pass

            try:
                if self.ser and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass

            try:
                if self.joystick:
                    self.joystick.quit()
                pygame.quit()
            except Exception:
                pass

            self.send_ui("stopped", "Bridge stopped. Safety command sent.")


# -----------------------------
# Main app class
# -----------------------------
class DroneApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("Drone Dashboard")
        self.geometry("1100x820")
        self.minsize(1000, 700)

        self.port_map = {}
        self.worker = None
        self.ui_queue = queue.Queue()
        self.command_queue = queue.Queue()

        # Tkinter variables
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.hz_var = tk.StringVar(value=str(DEFAULT_HZ))
        self.show_raw_var = tk.BooleanVar(value=False)

        self.connection_var = tk.StringVar(value="Disconnected")

        self.motor1_speed_var = tk.StringVar(value="0%")
        self.motor2_speed_var = tk.StringVar(value="0%")
        self.motor3_speed_var = tk.StringVar(value="0%")
        self.motor4_speed_var = tk.StringVar(value="0%")

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
        self.centered_var = tk.StringVar(value="Servo Control Active")

        self.new_ack_var = tk.StringVar(value="Waiting for data...")

        self.build_ui()
        self.refresh_ports()
        self.after(100, self.process_ui_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # -----------------------------
    # Building the UI
    # -----------------------------
    def build_ui(self):
        root = ttk.Frame(self, padding=15)
        root.pack(fill="both", expand=True)

        title = ttk.Label(root, text="Drone Dashboard", font=("Times New Roman", 22, "bold"))
        title.pack(anchor="w")

        subtitle = ttk.Label(root, text="PS4 controller -> bridge -> Arduino")
        subtitle.pack(anchor="w", pady=(0, 15))

        # Connection frame
        connection_box = ttk.LabelFrame(root, text="Connection", padding=10)
        connection_box.pack(fill="x")
        connection_box.columnconfigure(1, weight=1)

        ttk.Label(connection_box, text="USB / COM Port:").grid(row=0, column=0, sticky="w")

        self.port_combo = ttk.Combobox(connection_box, textvariable=self.port_var, width=45)
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=8)

        refresh_button = ttk.Button(connection_box, text="Refresh ports", command=self.refresh_ports)
        refresh_button.grid(row=0, column=2, padx=5)

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

        ttk.Label(connection_box, textvariable=self.connection_var).grid(
            row=2, column=1, columnspan=6, sticky="w"
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

        self.make_metric(motor_box, "Motor 2", self.motor2_speed_var, 0, 1)
        self.motor2_bar.grid(row=1, column=1, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor2_direction_var, font=("Times New Roman", 12)).grid(row=2, column=1, pady=(5, 0))

        self.make_metric(motor_box, "Motor 3", self.motor3_speed_var, 0, 2)
        self.motor3_bar.grid(row=1, column=2, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor3_direction_var, font=("Times New Roman", 12)).grid(row=2, column=2, pady=(5, 0))

        self.make_metric(motor_box, "Motor 4", self.motor4_speed_var, 0, 3)
        self.motor4_bar.grid(row=1, column=3, sticky="ew", padx=10)
        ttk.Label(motor_box, textvariable=self.motor4_direction_var, font=("Times New Roman", 12)).grid(row=2, column=3, pady=(5, 0))

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

        for i in range(3):
            safety_box.columnconfigure(i, weight=1)

        self.make_metric(safety_box, "Motor Lock", self.motor_lock_var, 0, 0)
        self.make_metric(safety_box, "Kill Switch", self.kill_var, 0, 1)
        self.make_metric(safety_box, "Servo Control", self.centered_var, 0, 2)

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
        ports = list(list_ports.comports())
        self.port_map.clear()
        display_values = []

        for port in ports:
            display_text = f"{port.device} - {port.description}"
            self.port_map[display_text] = port.device
            display_values.append(display_text)

        self.port_combo["values"] = display_values

        if display_values:
            self.port_var.set(display_values[0])
            self.log(f"Found {len(display_values)} serial port(s).")
        else:
            self.port_var.set("No Ports Found")
            self.log("No serial ports found.")

    def selected_port(self):
        selected = self.port_var.get().strip()
        return self.port_map.get(selected, selected)

    # -----------------------------
    # Connect button behavior
    # -----------------------------
    def connect(self):
        if self.worker and self.worker.is_alive():
            messagebox.showinfo("Already connected", "The bridge is already running.")
            return

        port = self.selected_port()

        if not port or port == "No Ports Found":
            messagebox.showerror("No port selected", "Plug in the Arduino and select the correct COM port first.")
            return

        try:
            baud = int(self.baud_var.get())
            hz = float(self.hz_var.get())
            if baud <= 0 or hz <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Bad settings", "Baud must be an integer and Hz must be a positive number.")
            return

        self.ui_queue = queue.Queue()
        self.command_queue = queue.Queue()

        self.worker = BridgeWorker(
            port=port,
            baud=baud,
            hz=hz,
            ui_queue=self.ui_queue,
            command_queue=self.command_queue,
            show_raw=self.show_raw_var.get(),
        )

        self.worker.start()

        self.connection_var.set(f"Connecting to {port}...")
        self.connect_button.config(state="disabled")
        self.disconnect_button.config(state="normal")
        self.kill_button.config(state="normal")
        self.log(f"Starting bridge on {port} @ {baud}, {hz} Hz.")

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
            self.command_queue.put("kill")
            self.connection_var.set("KILL requested...")
            self.disconnect_button.config(state="disabled")
            self.kill_button.config(state="disabled")
            self.log("KILL / STOP pressed. Sending safety command.")

    # -----------------------------
    # Update dashboard from Arduino ACK
    # -----------------------------
    def format_motor_direction(self, raw_direction):
        """Convert Arduino direction symbols into dashboard text."""
        if raw_direction in ["+", "CW"]:
            return "CW"
        if raw_direction in ["-", "CCW"]:
            return "CCW"
        return "0 Stopped"

    def update_motor_display(self, motor_num, pwm, direction):
        """Update one motor's percent bar and direction text."""
        motor_percent = max(0, min(100, int(pwm / 255 * 100)))

        if motor_num == 1:
            self.motor1_speed_var.set(f"{motor_percent}%")
            self.motor1_bar["value"] = motor_percent
            self.motor1_direction_var.set(self.format_motor_direction(direction))
        elif motor_num == 2:
            self.motor2_speed_var.set(f"{motor_percent}%")
            self.motor2_bar["value"] = motor_percent
            self.motor2_direction_var.set(self.format_motor_direction(direction))
        elif motor_num == 3:
            self.motor3_speed_var.set(f"{motor_percent}%")
            self.motor3_bar["value"] = motor_percent
            self.motor3_direction_var.set(self.format_motor_direction(direction))
        elif motor_num == 4:
            self.motor4_speed_var.set(f"{motor_percent}%")
            self.motor4_bar["value"] = motor_percent
            self.motor4_direction_var.set(self.format_motor_direction(direction))

    # -----------------------------
    # Update dashboard from Arduino ACK
    # -----------------------------
    def update_dashboard(self, ack_line):
        data = parse_ack(ack_line)

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

        self.motor_lock_var.set("Motor Locked" if data["motor_locked"] else "Motor Enabled")
        self.kill_var.set("Kill Active" if data["kill"] else "Kill Switch Off")
        self.centered_var.set("Servos Centered" if data["centered"] else "Servo Control Active")

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
                    self.log(message)
                elif kind == "stopped":
                    self.connection_var.set("Disconnected")
                    self.connect_button.config(state="normal")
                    self.disconnect_button.config(state="disabled")
                    self.kill_button.config(state="disabled")
                    self.log(message)
                elif kind == "error":
                    self.connection_var.set("Error / Disconnected")
                    self.connect_button.config(state="normal")
                    self.disconnect_button.config(state="disabled")
                    self.kill_button.config(state="disabled")
                    self.log(f"ERROR: {message}")
                    messagebox.showerror("Bridge error", message)
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
        if self.worker and self.worker.is_alive():
            self.command_queue.put("disconnect")
            self.worker.stop_event.set()
        self.destroy()


if __name__ == "__main__":
    app = DroneApp()
    app.mainloop()
