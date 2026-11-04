#!/bin/bash
# Launch the standalone YOLO (Hexagon NPU) pick-place app. Output streams to
# this terminal — Ctrl-C to stop. Pass --mode {pickplace,ball} and other
# yolo_pickplace.py args after the script name.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

exec stdbuf -oL -eL python -u yolo_pickplace.py "$@"
