#!/bin/bash
# Launch the teach-pose helper (torque off, read servo positions by hand).
# Run this from the board. Stop main.py first or it will contend for USB.
# Usage: ./scripts/teach.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR" || exit 1

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

LOG="$ROOT_DIR/teach.log"
echo "===== TEACH $(date -u +%FT%TZ) =====" | tee "$LOG"

exec stdbuf -oL -eL python -u calibration/teach_pose.py "$@" 2>&1 | tee -a "$LOG"
