#!/bin/bash
# Launch the camera-gripper calibrator (view pose + grab reach) — a browser
# page at http://<board-ip>:8000/. Same calibrator the `calibrate_offset`
# cloud command opens. Stop main.py first or it will contend for USB + the
# camera.
# Usage: ./scripts/calibrate-offset.sh [--port 8000] [--camera N]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR" || exit 1

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

LOG="$ROOT_DIR/calibrate_offset.log"
echo "===== CALIB-OFFSET $(date -u +%FT%TZ) =====" | tee "$LOG"

exec stdbuf -oL -eL python -u calibration/browser_calibrate_offset.py "$@" 2>&1 | tee -a "$LOG"
