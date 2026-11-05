#!/bin/bash
# Launch the teach-grab-depth helper.
# Stop the running yolo demo first or it will hold the camera.
# Usage:  ./teach_grab.sh --model model/ball_best.tflite --web-port 8080

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

exec stdbuf -oL -eL python -u teach_grab_depth.py "$@"
