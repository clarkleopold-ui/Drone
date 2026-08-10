#!/bin/zsh

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [[ ! -x ".venv/bin/python" ]]; then
  print "The one-time setup has not been run yet."
  print "Starting BUILD_AMBATUDRONE_MAC.command..."
  exec "$SCRIPT_DIR/BUILD_AMBATUDRONE_MAC.command"
fi

exec "$SCRIPT_DIR/.venv/bin/python" "$SCRIPT_DIR/droneControl4Motor.py"
