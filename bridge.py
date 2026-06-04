"""
DualShock 4 -> Computer -> Arduino bridge

Install:
    pip install pygame pyserial

Run examples:
    python bench_fixed.py --port COM5
    python bench_fixed.py --port /dev/cu.usbmodemXXXX

Optional debug:
    python bench_fixed.py --port COM5 --show-raw
"""

import argparse
import json
import time

import pygame
import serial
import os

STOP_FILE = "stop_bridge.txt"

def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_axis(raw, deadzone=0.08):
    """Convert pygame joystick axis float [-1.0, 1.0] to int [-100, 100]."""
    if abs(raw) < deadzone:
        return 0
    return clamp(int(raw * 100), -100, 100)


def get_axis_safe(joystick, axis_index, default=0.0):
    """Avoid crashing if a controller/driver exposes fewer axes than expected."""
    if axis_index < joystick.get_numaxes():
        return joystick.get_axis(axis_index)
    return default


def get_button_safe(joystick, button_index, default=0):
    """Avoid crashing if a controller/driver exposes fewer buttons than expected."""
    if button_index < joystick.get_numbuttons():
        return int(joystick.get_button(button_index))
    return default


def main():
    parser = argparse.ArgumentParser(description="DualShock 4 -> Arduino bridge")
    parser.add_argument("--port", required=True, help="Arduino serial port, e.g. COM5 or /dev/cu.usbmodemXXXX")
    parser.add_argument("--baud", type=int, default=115200, help="Must match Serial.begin() in Arduino")
    parser.add_argument("--hz", type=float, default=25.0, help="How many command packets per second")
    parser.add_argument("--show-raw", action="store_true", help="Print raw axis/button counts for debugging")
    args = parser.parse_args()

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected. Connect the PS4 controller to the computer and retry.")

    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Connected joystick: {joystick.get_name()}")
    print(f"Axes: {joystick.get_numaxes()} | Buttons: {joystick.get_numbuttons()} | Hats: {joystick.get_numhats()}")

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    time.sleep(2.0)  # Arduino resets when serial opens
    ser.reset_input_buffer()

    print(f"Sending commands to Arduino on {args.port} @ {args.baud}")
    print("Controls: hold R1 for motor enable, left stick Y = throttle, right stick = servos, X = kill motor, L1 = center servos")

    period = 1.0 / args.hz
    last_status_print = 0.0


    try:
        while True:
            if os.path.exists(STOP_FILE):
                print("Stop requested from dashboard.")

                safety_payload = {
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

                ser.write((json.dumps(safety_payload, separators=(",", ":")) + "\n").encode("utf-8"))
                time.sleep(0.1)

                os.remove(STOP_FILE)
                break

            pygame.event.pump()

            # Common DualShock 4 mappings in pygame:
            # axis 0 = left stick X
            # axis 1 = left stick Y, negative when pushed upward
            # axis 2 = right stick X on many systems
            # axis 3 or 5 = right stick Y depending on OS/driver
            # This code prefers axis 3 for RY, but falls back to 5 if needed.
            raw_lx = get_axis_safe(joystick, 0)
            raw_ly = get_axis_safe(joystick, 1)
            raw_rx = get_axis_safe(joystick, 2)
            raw_ry = get_axis_safe(joystick, 3, get_axis_safe(joystick, 5))

            payload = {
                "lx": normalize_axis(raw_lx),
                "ly": normalize_axis(-raw_ly),  # push up = positive throttle
                "rx": normalize_axis(raw_rx),
                "ry": normalize_axis(-raw_ry),
                "cross": get_button_safe(joystick, 0),
                "circle": get_button_safe(joystick, 1),
                "square": get_button_safe(joystick, 2),
                "triangle": get_button_safe(joystick, 3),
                "l1": get_button_safe(joystick, 4),
                "r1": get_button_safe(joystick, 10),
            }

            line = json.dumps(payload, separators=(",", ":"))
            ser.write((line + "\n").encode("utf-8"))

            while ser.in_waiting:
                ack = ser.readline().decode("utf-8", errors="ignore").strip()
                print(ack)

                with open("latest_status.txt", "w") as f:
                    f.write(ack)

            if args.show_raw and time.time() - last_status_print > 0.5:
                axes = [round(get_axis_safe(joystick, i), 3) for i in range(joystick.get_numaxes())]
                buttons = [get_button_safe(joystick, i) for i in range(joystick.get_numbuttons())]
                print(f"RAW axes={axes} buttons={buttons}")
                last_status_print = time.time()

            time.sleep(period)

    except KeyboardInterrupt:
        print("Stopping bridge")
    finally:
        try:
            # Send one final safety packet: motor disabled, servos centered.
            ser.write(b'{"lx":0,"ly":0,"rx":0,"ry":0,"cross":1,"circle":0,"square":0,"triangle":0,"l1":1,"r1":0}\n')
            time.sleep(0.05)
        except Exception:
            pass
        ser.close()
        joystick.quit()
        pygame.quit()


if __name__ == "__main__":
    main()
