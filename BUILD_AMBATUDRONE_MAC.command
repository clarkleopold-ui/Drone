#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$SCRIPT_DIR"

fail() {
  print
  print "Build stopped: $1"
  print
  read -r "?Press Return to close this window..."
  exit 1
}

PYTHON_CMD=""
PYTHON_CANDIDATES=(
  "/Library/Frameworks/Python.framework/Versions/3.14/bin/python3"
  "/Library/Frameworks/Python.framework/Versions/3.13/bin/python3"
  "/Library/Frameworks/Python.framework/Versions/3.12/bin/python3"
  "/Library/Frameworks/Python.framework/Versions/3.11/bin/python3"
  "python3.14"
  "python3.13"
  "python3.12"
  "python3.11"
  "python3"
)

for candidate in "${PYTHON_CANDIDATES[@]}"; do
  if [[ "$candidate" == /* ]]; then
    [[ -x "$candidate" ]] || continue
    resolved="$candidate"
  else
    resolved="$(command -v "$candidate" 2>/dev/null || true)"
    [[ -n "$resolved" ]] || continue
  fi

  if "$resolved" -c \
    'import sys; raise SystemExit(0 if (3, 11) <= sys.version_info[:2] < (3, 15) else 1)' \
    >/dev/null 2>&1; then
    PYTHON_CMD="$resolved"
    break
  fi
done

if [[ -z "$PYTHON_CMD" ]]; then
  DETECTED_VERSION="$(python3 --version 2>&1 || print 'not installed')"
  fail "Python 3.11 through 3.14 is required. Detected: $DETECTED_VERSION

Install the macOS installer for Python 3.13 from:
https://www.python.org/downloads/release/python-31314/

Then run this file again."
fi

"$PYTHON_CMD" -c 'import tkinter' >/dev/null 2>&1 || fail \
  "The selected Python installation does not include Tkinter. Install Python 3.13 from python.org, then run this file again."

print "Using $("$PYTHON_CMD" --version) at $PYTHON_CMD"

print "Creating the AmbatuDrone Mac environment..."
"$PYTHON_CMD" -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip setuptools wheel
python -m pip install -r requirements-mac.txt

print
print "Running the hardware-free protocol test..."
python droneControl4Motor.py --self-test

print
print "Building AmbatuDrone.app..."
python -m PyInstaller \
  --noconfirm \
  --clean \
  --windowed \
  --name AmbatuDrone \
  --osx-bundle-identifier com.ambatudrone.dashboard \
  --collect-all pygame \
  --hidden-import serial.tools.list_ports \
  droneControl4Motor.py

APP_PATH="$SCRIPT_DIR/dist/AmbatuDrone.app"
[[ -d "$APP_PATH" ]] || fail "PyInstaller finished, but AmbatuDrone.app was not created."

print
print "Build complete:"
print "$APP_PATH"
print
print "The app will open in Demo Mode. No controller or flight hardware is required."
open "$APP_PATH"
open "$SCRIPT_DIR/dist"

print
read -r "?Press Return to close this window..."
