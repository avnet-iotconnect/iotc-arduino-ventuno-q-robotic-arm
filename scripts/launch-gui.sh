#!/bin/bash
# Wrapper that sets the Wayland environment so a GUI command launched from
# SSH (or a desktop launcher) renders on the board's HDMI display (the
# VENTUNO Q's GNOME session for the logged-in `arduino` user).
# Output streams to this terminal — Ctrl-C to stop.
#
# Usage:
#   ./scripts/launch-gui.sh calibrate-ball     # opens ball calibrator on the HDMI display
#   ./scripts/launch-gui.sh calibrate-box      # opens box calibrator (writes box_color.json)
#   ./scripts/launch-gui.sh calibrate-offset   # camera-gripper calibrator (browser page on :8000)
#   ./scripts/launch-gui.sh teach-pose         # opens scan-pose teacher
#   ./scripts/launch-gui.sh ball               # runs the ball-follow demo
#   ./scripts/launch-gui.sh pickplace          # runs the pick-and-place demo
#   ./scripts/launch-gui.sh asl                # runs the ASL gesture demo
#   ./scripts/launch-gui.sh shell              # opens a terminal in the repo root on the HDMI desktop
#
# Pass extra args after the command name and they're forwarded:
#   ./scripts/launch-gui.sh ball --camera 2
#
# The USB camera is auto-detected (camera_settings.find_brio_index). On the
# VENTUNO Q, /dev/video0, video1, video32 and video33 are Qualcomm
# platform/codec nodes, not cameras — pass --camera only to override.
#
# Only one process can hold the camera + xarm USB at a time — don't launch
# two demos concurrently.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"
cd "$ROOT_DIR"

# Python env ($PYTHON) + NPU runtime paths for this board.
source "$SCRIPT_DIR/board-env.sh"

# Send windows (cv2.imshow) to the HDMI desktop (GNOME session).
if ! board_use_display; then
    echo "WARNING: $(id -un) is not logged in on the HDMI desktop —" \
         "log in there first, or use --headless --web-port 8080." >&2
fi

# Calibrators have an interactive `input()` safety prompt before dropping
# servo torque. When launched from a desktop icon, there's no tty,
# so input() raises EOFError and the script crashes. Detect non-interactive
# stdin (panel-launched, cron, etc.) and auto-pass --no-prompt — the user at
# the panel is by definition standing at the board with hand on the arm.
NO_PROMPT_ARG=""
if [ ! -t 0 ]; then
    NO_PROMPT_ARG="--no-prompt"
fi

cmd="$1"; shift || true

case "$cmd" in
  calibrate-ball)
    exec stdbuf -oL -eL "$PYTHON" -u calibration/ball_calibrate.py --output ball_color.json $NO_PROMPT_ARG "$@"
    ;;
  calibrate-box)
    exec stdbuf -oL -eL "$PYTHON" -u calibration/ball_calibrate.py --output box_color.json $NO_PROMPT_ARG "$@"
    ;;
  calibrate-offset|calibrate-cam-offset)
    exec stdbuf -oL -eL "$PYTHON" -u calibration/browser_calibrate_offset.py $NO_PROMPT_ARG "$@"
    ;;
  teach-pose|teach)
    exec stdbuf -oL -eL "$PYTHON" -u calibration/teach_pose.py "$@"
    ;;
  ball)
    exec stdbuf -oL -eL "$PYTHON" -u main.py --mode ball "$@"
    ;;
  pickplace)
    exec stdbuf -oL -eL "$PYTHON" -u main.py --mode pickplace "$@"
    ;;
  asl)
    exec stdbuf -oL -eL "$PYTHON" -u main.py --mode asl "$@"
    ;;
  shell)
    # Terminal opened in the repo root — handy for desktop-launched calibration.
    exec gnome-terminal --working-directory="$ROOT_DIR"
    ;;
  *)
    echo "Usage: $0 {calibrate-ball|calibrate-box|calibrate-offset|teach-pose|ball|pickplace|asl|shell} [args...]" >&2
    exit 2
    ;;
esac
