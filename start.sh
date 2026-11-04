#!/bin/bash
# Launch the IoTConnect XArm gesture demo.
# Run from the board's local HDMI terminal so the OpenCV preview window can render.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

LOG="$SCRIPT_DIR/run.log"
echo "===== START $(date -u +%FT%TZ) =====" | tee "$LOG"
echo "Logging to $LOG (tail -f from another host: ssh arduino@<board> tail -f $LOG)" | tee -a "$LOG"

# stdbuf so python flushes line-by-line into the log
exec stdbuf -oL -eL python -u main.py "$@" 2>&1 | tee -a "$LOG"
