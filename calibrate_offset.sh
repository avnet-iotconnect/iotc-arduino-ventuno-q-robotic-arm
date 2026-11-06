#!/bin/bash
# Launch the camera-gripper calibrator (view pose + grab reach) — a browser
# page at http://<board-ip>:8000/. Same calibrator the `calibrate_offset`
# cloud command opens. Stop main.py first or it will contend for USB + the
# camera.
# Usage: ./calibrate_offset.sh [--port 8000] [--camera N]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

LOG="$SCRIPT_DIR/calibrate_offset.log"
echo "===== CALIB-OFFSET $(date -u +%FT%TZ) =====" | tee "$LOG"

exec stdbuf -oL -eL python -u browser_calibrate_offset.py "$@" 2>&1 | tee -a "$LOG"
