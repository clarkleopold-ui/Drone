#!/usr/bin/env python3
"""
Propeller-free serial bench test for bridgeMotorControl_July_20_corrected.ino.

This script bypasses the PS4 controller and Tkinter dashboard. It sends the
same JSON packets as the real bridge, reads the Mega's ACK telemetry, and
provides a small keyboard command interface.

Dependency:
    py -m pip install pyserial

Examples:
    py bridge_bench_test.py
    py bridge_bench_test.py --port COM5
    py bridge_bench_test.py --self-test
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    import serial
    from serial import SerialException
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

    class SerialException(Exception):
        """Fallback used only so --self-test can run without pyserial."""


DEFAULT_BAUD = 115200
SEND_HZ = 25.0
STARTUP_TIMEOUT_S = 12.0
FIRST_ACK_TIMEOUT_S = 3.0
TELEMETRY_TIMEOUT_S = 1.0
MAX_BENCH_THROTTLE = 25
MAX_RX_BUFFER = 4096

MOTOR_RE = re.compile(
    r"motor_pwm=(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)"
)


def safe_payload() -> dict[str, int]:
    """Return an immediate motor-kill packet using the Mega's expected keys."""
    return {
        "lx": 0,
        "ly": 0,
        "rx": 0,
        "ry": 0,
        "cross": 1,
        "circle": 0,
        "square": 0,
        "triangle": 0,
        "l1": 0,
        "r1": 0,
    }


def encode_packet(payload: dict[str, int]) -> bytes:
    """Encode one compact, newline-terminated command for the Mega."""
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode("ascii")


@dataclass(frozen=True)
class Ack:
    raw: str
    motors: tuple[int, int, int, int]
    lx: int
    ly: int
    rx: int
    ry: int
    roll: float
    pitch: float
    yaw: float
    yaw_rate: float
    kill: bool
    motor_locked: bool
    throttle_not_low: bool
    failsafe: bool
    imu_error: bool


def _read_int(line: str, key: str, default: int = 0) -> int:
    match = re.search(rf"\b{re.escape(key)}=(-?\d+)", line)
    return int(match.group(1)) if match else default


def _read_float_after_label(line: str, label: str, default: float = 0.0) -> float:
    match = re.search(
        rf"{re.escape(label)}(?:\s*\[[^\]]*\])?\s*:\s*(-?\d+(?:\.\d+)?)",
        line,
    )
    return float(match.group(1)) if match else default


def parse_ack(line: str) -> Optional[Ack]:
    """Parse one corrected-Mega ACK line; ignore startup/debug text."""
    if not line.startswith("ACK "):
        return None

    motor_match = MOTOR_RE.search(line)
    if not motor_match:
        return None

    return Ack(
        raw=line,
        motors=tuple(int(motor_match.group(i)) for i in range(1, 5)),
        lx=_read_int(line, "lx"),
        ly=_read_int(line, "ly"),
        rx=_read_int(line, "rx"),
        ry=_read_int(line, "ry"),
        roll=_read_float_after_label(line, "Roll"),
        pitch=_read_float_after_label(line, "Pitch"),
        yaw=_read_float_after_label(line, "Yaw"),
        yaw_rate=_read_float_after_label(line, "Yaw Rate"),
        kill="KILL" in line,
        motor_locked="MOTOR_LOCKED" in line,
        throttle_not_low="THROTTLE_NOT_LOW" in line,
        failsafe="FAILSAFE" in line,
        imu_error="IMU_ERROR" in line,
    )


class BenchBridge:
    def __init__(self, port: str, baud: int) -> None:
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None

        self.payload = safe_payload()
        self.payload_lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.ack_lock = threading.Lock()

        self.stop_event = threading.Event()
        self.ack_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

        self.rx_buffer = bytearray()
        self.last_ack: Optional[Ack] = None
        self.last_ack_time = 0.0
        self.last_info_line = ""
        self.error: Optional[BaseException] = None
        self.locally_armed = False
        self.link_timeout_kill_applied = False

    def open(self) -> None:
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=0.25,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        print(
            f"Opened {self.port} at {self.baud} baud.\n"
            "Keep the MPU-6050 still while the Mega starts..."
        )
        self._wait_for_mega_startup()

        self.stop_event.clear()
        self.thread = threading.Thread(
            target=self._communication_loop,
            name="bench-serial-bridge",
            daemon=True,
        )
        self.thread.start()

        if not self.ack_event.wait(FIRST_ACK_TIMEOUT_S):
            detail = f" Last line: {self.last_info_line}" if self.last_info_line else ""
            raise RuntimeError(
                "The serial port opened, but no valid ACK arrived after safe "
                f"commands were sent.{detail}"
            )

    def _wait_for_mega_startup(self) -> None:
        """Avoid filling the Mega's RX buffer while its IMU is calibrating."""
        assert self.ser is not None
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        startup_buffer = bytearray()

        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            chunk = self.ser.read(waiting if waiting else 1)
            if chunk:
                startup_buffer.extend(chunk)

            while b"\n" in startup_buffer:
                raw_line, _, startup_buffer = startup_buffer.partition(b"\n")
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue

                self.last_info_line = line
                if line == "Arduino bridge ready" or parse_ack(line) is not None:
                    print(f"Mega detected: {line}")
                    return

        raise RuntimeError(
            "The Mega did not finish startup within 12 seconds. Check the COM "
            "port, USB cable, uploaded sketch, and MPU-6050 wiring."
        )

    def _communication_loop(self) -> None:
        assert self.ser is not None
        period = 1.0 / SEND_HZ
        next_send = time.monotonic()

        try:
            while not self.stop_event.is_set():
                now = time.monotonic()

                if now >= next_send:
                    self._send_current_payload()
                    next_send = now + period

                waiting = self.ser.in_waiting
                if waiting:
                    self._consume_rx(self.ser.read(waiting))

                if (
                    self.last_ack_time > 0
                    and now - self.last_ack_time > TELEMETRY_TIMEOUT_S
                    and not self.link_timeout_kill_applied
                ):
                    self.kill()
                    self.link_timeout_kill_applied = True

                time.sleep(0.002)
        except (SerialException, OSError) as exc:
            self.error = exc
            self.kill()
            self.stop_event.set()

    def _send_current_payload(self) -> None:
        assert self.ser is not None
        with self.payload_lock:
            packet = encode_packet(dict(self.payload))
        with self.write_lock:
            self.ser.write(packet)

    def _consume_rx(self, chunk: bytes) -> None:
        self.rx_buffer.extend(chunk)

        if len(self.rx_buffer) > MAX_RX_BUFFER:
            self.rx_buffer.clear()
            return

        while b"\n" in self.rx_buffer:
            raw_line, _, remainder = self.rx_buffer.partition(b"\n")
            self.rx_buffer = bytearray(remainder)
            line = raw_line.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            ack = parse_ack(line)
            if ack is None:
                self.last_info_line = line
                continue

            with self.ack_lock:
                self.last_ack = ack
                self.last_ack_time = time.monotonic()
            self.link_timeout_kill_applied = False
            self.ack_event.set()

    def get_ack(self) -> Optional[Ack]:
        with self.ack_lock:
            return self.last_ack

    def set_payload(self, **changes: int) -> None:
        with self.payload_lock:
            self.payload.update(changes)

    def kill(self) -> None:
        with self.payload_lock:
            self.payload = safe_payload()
        self.locally_armed = False

    def arm_at_zero(self) -> bool:
        """Perform the Mega's required disarm-low-arm sequence."""
        self.kill()
        self.set_payload(cross=0, r1=0, ly=0)
        time.sleep(0.65)
        self.set_payload(cross=0, r1=1, ly=0)
        time.sleep(0.35)

        ack = self.get_ack()
        if ack is None:
            return False
        if ack.imu_error or ack.failsafe or ack.throttle_not_low:
            self.kill()
            return False

        self.locally_armed = not ack.motor_locked
        return self.locally_armed

    def set_throttle(self, percent: int) -> None:
        if not self.locally_armed:
            raise RuntimeError("Run 'arm' before applying throttle.")
        if not 0 <= percent <= MAX_BENCH_THROTTLE:
            raise ValueError(
                f"Bench throttle must be between 0 and {MAX_BENCH_THROTTLE}%."
            )
        self.set_payload(cross=0, r1=1, ly=percent)

    def close(self) -> None:
        if self.ser is None:
            return

        # Send several explicit kill packets before closing the port.
        self.kill()
        for _ in range(5):
            try:
                self._send_current_payload()
            except (SerialException, OSError):
                break
            time.sleep(0.04)

        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.ser.is_open:
            self.ser.close()


def format_ack(ack: Optional[Ack], age: Optional[float] = None) -> str:
    if ack is None:
        return "No valid ACK has been received."

    flags = []
    if ack.kill:
        flags.append("KILL")
    if ack.motor_locked:
        flags.append("MOTOR_LOCKED")
    if ack.throttle_not_low:
        flags.append("THROTTLE_NOT_LOW")
    if ack.failsafe:
        flags.append("FAILSAFE")
    if ack.imu_error:
        flags.append("IMU_ERROR")
    if not flags:
        flags.append("ARMED / NO SAFETY FLAGS")

    age_text = f" | ACK age={age:.2f}s" if age is not None else ""
    return (
        f"Motors: {ack.motors} us\n"
        f"Axes: lx={ack.lx} ly={ack.ly} rx={ack.rx} ry={ack.ry}\n"
        f"Attitude: roll={ack.roll:.2f}° pitch={ack.pitch:.2f}° "
        f"yaw={ack.yaw:.2f}° yaw-rate={ack.yaw_rate:.2f}°/s\n"
        f"State: {', '.join(flags)}{age_text}"
    )


def choose_port() -> str:
    assert list_ports is not None
    ports = list(list_ports.comports())
    if not ports:
        raise RuntimeError(
            "No serial ports were found. Connect the Mega and close Arduino "
            "Serial Monitor before trying again."
        )

    print("Available serial ports:")
    for index, port in enumerate(ports, start=1):
        description = port.description or "No description"
        print(f"  {index}. {port.device} — {description}")

    while True:
        answer = input("Select a port number: ").strip()
        try:
            selected = int(answer)
            if 1 <= selected <= len(ports):
                return ports[selected - 1].device
        except ValueError:
            pass
        print("Enter one of the listed numbers.")


def print_help() -> None:
    print(
        "\nCommands:\n"
        "  status       Show the newest Mega ACK\n"
        "  arm          Clear kill and arm at 0% throttle\n"
        f"  t N          Set throttle to N% (0-{MAX_BENCH_THROTTLE} only)\n"
        "  stop         Return throttle to 0% but remain armed\n"
        "  kill         Immediate motor kill and disarm\n"
        "  raw          Print the complete newest ACK line\n"
        "  help         Show this command list\n"
        "  quit         Kill motors, close serial, and exit\n"
    )


def interactive_test(bridge: BenchBridge) -> None:
    print("\nBridge verified. The script is in KILL state.")
    print_help()
    print(format_ack(bridge.get_ack()))

    while True:
        if bridge.error is not None:
            raise RuntimeError(f"Serial connection failed: {bridge.error}")

        command = input("\nbench> ").strip().lower()
        if not command:
            continue

        if command in {"quit", "q", "exit"}:
            return

        if command in {"help", "h", "?"}:
            print_help()
            continue

        if command in {"status", "s"}:
            ack = bridge.get_ack()
            age = (
                time.monotonic() - bridge.last_ack_time
                if bridge.last_ack_time
                else None
            )
            print(format_ack(ack, age))
            continue

        if command == "raw":
            ack = bridge.get_ack()
            print(ack.raw if ack else "No valid ACK has been received.")
            continue

        if command in {"kill", "k"}:
            bridge.kill()
            time.sleep(0.15)
            print("KILL sent. All commanded motor outputs should be 1000 us.")
            print(format_ack(bridge.get_ack()))
            continue

        if command in {"stop", "0"}:
            if bridge.locally_armed:
                bridge.set_throttle(0)
                time.sleep(0.15)
                print("Throttle returned to 0%.")
                print(format_ack(bridge.get_ack()))
            else:
                bridge.kill()
                print("Already disarmed; KILL remains active.")
            continue

        if command == "arm":
            print(
                "Before arming: confirm all propellers are removed, the frame "
                "is secure, and wires cannot touch the motors."
            )
            if input("Type ARM to continue: ").strip() != "ARM":
                print("Arming cancelled.")
                continue

            if bridge.arm_at_zero():
                print("Mega reports ARMED at 0% throttle.")
            else:
                print("Arming failed. Check the safety flags below:")
            print(format_ack(bridge.get_ack()))
            continue

        throttle_match = re.fullmatch(r"(?:t|throttle)\s+(\d{1,3})", command)
        if throttle_match:
            try:
                percent = int(throttle_match.group(1))
                bridge.set_throttle(percent)
                time.sleep(0.2)
                print(f"Bench throttle command: {percent}%.")
                print(format_ack(bridge.get_ack()))
            except (RuntimeError, ValueError) as exc:
                print(exc)
            continue

        print("Unknown command. Type 'help' to see the command list.")


def run_self_test() -> None:
    sample = (
        "ACK motor_pwm=1000 1010 1020 1030 "
        "lx=1 ly=2 rx=3 ry=4 "
        "Roll [°]: 1.25 Pitch [°]: -2.50 Yaw [°]: 3.75 "
        "Yaw Rate [°/s]: -4.50 MOTOR_LOCKED"
    )
    ack = parse_ack(sample)
    assert ack is not None
    assert ack.motors == (1000, 1010, 1020, 1030)
    assert (ack.lx, ack.ly, ack.rx, ack.ry) == (1, 2, 3, 4)
    assert (ack.roll, ack.pitch, ack.yaw, ack.yaw_rate) == (
        1.25,
        -2.5,
        3.75,
        -4.5,
    )
    assert ack.motor_locked
    assert parse_ack("Arduino bridge ready") is None

    packet = encode_packet(safe_payload())
    decoded = json.loads(packet.decode("ascii"))
    assert decoded == safe_payload()
    assert packet.endswith(b"\n")
    assert len(packet) < 180
    print("Self-test passed: packet encoding and ACK parsing are compatible.")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Propeller-free Arduino Mega bridge bench test"
    )
    parser.add_argument("--port", help="Serial port, for example COM5")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Test packet/parser logic without opening hardware",
    )
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return 0

    if serial is None:
        print("Missing dependency: pyserial")
        print("Install it with: py -m pip install pyserial")
        return 1

    print(
        "PROPELLER-FREE TEST ONLY\n"
        "This script starts and exits with an explicit motor-kill command."
    )

    try:
        port = args.port or choose_port()
        bridge = BenchBridge(port, args.baud)
        try:
            bridge.open()
            interactive_test(bridge)
        finally:
            bridge.close()
            print("\nFinal KILL packets sent. Serial port closed.")
        return 0
    except KeyboardInterrupt:
        print("\nInterrupted.")
        return 130
    except (RuntimeError, SerialException, OSError) as exc:
        print(f"\nERROR: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
