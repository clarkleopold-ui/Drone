import re
import time
import streamlit as st
import os

st.set_page_config(page_title="Drone Dashboard", layout="wide")

st.title("Drone Dashboard")

if st.button("KILL / STOP BRIDGE"):
    with open("stop_bridge.txt", "w") as f:
        f.write("stop")

    st.error("Kill command sent. Bridge should stop and motor should shut off.")

placeholder = st.empty()

def parse_ack(line):
    values = {}

    for key in ["motor_pwm", "servo1", "servo2", "lx", "ly", "rx", "ry"]:
        match = re.search(rf"{key}=(-?\d+)", line)
        values[key] = int(match.group(1)) if match else 0

    dir_match = re.search(r"motor_dir=([+\-0])", line)
    values["motor_dir"] = dir_match.group(1) if dir_match else "0"

    values["motor_locked"] = "MOTOR_LOCKED" in line
    values["kill"] = "KILL" in line
    values["centered"] = "SERVOS_CENTERED" in line

    return values

while True:
    try:
        with open("latest_status.txt", "r") as f:
            line = f.read().strip()
    except FileNotFoundError:
        line = "Waiting for bridge.py data..."

    data = parse_ack(line)

    with placeholder.container():
        st.subheader("Live Status")

        col1, col2, col3 = st.columns(3)

        motor_pwm = data["motor_pwm"]
        motor_percent = max(0, min(100, int(motor_pwm / 255 * 100)))
        motor_dir = data["motor_dir"]

        if motor_dir == "+":
            direction_text = "+ Clockwise"
        elif motor_dir == "-":
            direction_text = "- Counterclockwise"
        else:
            direction_text = "0 Stopped"

        col1.metric("Motor Speed", f"{motor_percent}%")
        col1.progress(motor_percent / 100)
        col1.metric("Direction", direction_text)

        col2.metric("Servo 1", f"{data['servo1']}°")
        col2.progress(max(0, min(1, data["servo1"] / 180)))

        col3.metric("Servo 2", f"{data['servo2']}°")
        col3.progress(max(0, min(1, data["servo2"] / 180)))

        st.divider()

        c1, c2, c3, c4 = st.columns(4)
        c1.metric("LX", data["lx"])
        c2.metric("LY", data["ly"])
        c3.metric("RX", data["rx"])
        c4.metric("RY", data["ry"])

        st.divider()

        s1, s2, s3 = st.columns(3)

        if data["motor_locked"]:
            s1.warning("Motor Locked")
        else:
            s1.success("Motor Enabled")

        if data["kill"]:
            s2.error("Kill Active")
        else:
            s2.success("Kill Off")

        if data["centered"]:
            s3.info("Servos Centered")
        else:
            s3.success("Servo Control Active")

        st.code(line)

    time.sleep(0.1)