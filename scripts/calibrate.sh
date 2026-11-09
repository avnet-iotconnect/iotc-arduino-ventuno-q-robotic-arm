#!/bin/bash
# Launch the ball-color HSV calibrator.
# Run from the board's local HDMI terminal so the OpenCV preview window can render.
# Stop main.py first — this script grabs the USB arm bus to release wrist torque.
# Usage: ./scripts/calibrate.sh [--camera N] [--width W] [--height H]

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR" || exit 1

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

LOG="$ROOT_DIR/calibrate.log"
echo "===== CALIBRATE $(date -u +%FT%TZ) =====" | tee "$LOG"

exec stdbuf -oL -eL python -u calibration/ball_calibrate.py "$@" 2>&1 | tee -a "$LOG"
