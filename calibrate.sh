#!/bin/bash
# Launch the ball-color HSV calibrator.
# Run from the board's local HDMI terminal so the OpenCV preview window can render.
# Stop main.py first — this script grabs the USB arm bus to release wrist torque.
# Usage: ./calibrate.sh [--camera N] [--width W] [--height H]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

LOG="$SCRIPT_DIR/calibrate.log"
echo "===== CALIBRATE $(date -u +%FT%TZ) =====" | tee "$LOG"

exec stdbuf -oL -eL python -u ball_calibrate.py "$@" 2>&1 | tee -a "$LOG"
