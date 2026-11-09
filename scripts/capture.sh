#!/bin/bash
# Capture ball training images from the wrist camera. Stop the demo first so
# the camera is free. Watch the live preview at http://<board-ip>:8080/.
# Ctrl-C to stop. Args pass through to tools/dataset/capture_dataset.py.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR" || exit 1

# Python env + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

exec stdbuf -oL -eL python -u tools/dataset/capture_dataset.py "$@"
