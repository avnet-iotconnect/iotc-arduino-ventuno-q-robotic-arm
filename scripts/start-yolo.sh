#!/bin/bash
# Launch the standalone YOLO (Hexagon NPU) pick-place app. Output streams to
# this terminal — Ctrl-C to stop. Pass --mode {pickplace,ball} and other
# tools/yolo_pickplace.py args after the script name.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR" || exit 1

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

exec stdbuf -oL -eL python -u tools/yolo_pickplace.py "$@"
