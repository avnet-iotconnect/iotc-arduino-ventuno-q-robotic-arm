#!/bin/bash
# Capture ball training images from the wrist camera. Stop the demo first so
# the camera is free. Watch the live preview at http://<board-ip>:8080/.
# Ctrl-C to stop. Args pass through to capture_dataset.py.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

exec stdbuf -oL -eL python -u capture_dataset.py "$@"
