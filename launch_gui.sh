#!/bin/bash
# Wrapper that sets the Wayland environment so a GUI command launched from
# SSH (or a desktop launcher) renders on the board's HDMI display (the
# VENTUNO Q's GNOME session for the logged-in `arduino` user).
# Output streams to this terminal — Ctrl-C to stop.
#
# Usage:
#   ./launch_gui.sh calibrate-ball     # opens ball calibrator on the HDMI display
#   ./launch_gui.sh calibrate-box      # opens box calibrator (writes box_color.json)
#   ./launch_gui.sh calibrate-offset   # camera-gripper offset calibrator (cv2 window)
#   ./launch_gui.sh teach-pose         # opens scan-pose teacher
#   ./launch_gui.sh ball               # runs the ball-follow demo
#   ./launch_gui.sh pickplace          # runs the pick-and-place demo
#   ./launch_gui.sh asl                # runs the ASL gesture demo
#   ./launch_gui.sh shell              # opens a terminal in this dir on the HDMI desktop
#
# Pass extra args after the command name and they're forwarded:
#   ./launch_gui.sh ball --camera 2
#
# The USB camera is auto-detected (camera_settings.find_brio_index). On the
# VENTUNO Q, /dev/video0, video1, video32 and video33 are Qualcomm
# platform/codec nodes, not cameras — pass --camera only to override.
#
# Only one process can hold the camera + xarm USB at a time — don't launch
# two demos concurrently.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python env ($PYTHON) + NPU runtime paths for this board.
source "$SCRIPT_DIR/board_env.sh"

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
    exec stdbuf -oL -eL "$PYTHON" -u ball_calibrate.py --output ball_color.json $NO_PROMPT_ARG "$@"
    ;;
  calibrate-box)
    exec stdbuf -oL -eL "$PYTHON" -u ball_calibrate.py --output box_color.json $NO_PROMPT_ARG "$@"
    ;;
  calibrate-offset|calibrate-cam-offset)
    exec stdbuf -oL -eL "$PYTHON" -u calibrate_cam_offset.py "$@"
    ;;
  teach-pose|teach)
    exec stdbuf -oL -eL "$PYTHON" -u teach_pose.py "$@"
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
    # Terminal opened in the project dir — handy for desktop-launched calibration.
    exec gnome-terminal --working-directory="$SCRIPT_DIR"
    ;;
  *)
    echo "Usage: $0 {calibrate-ball|calibrate-box|calibrate-offset|teach-pose|ball|pickplace|asl|shell} [args...]" >&2
    exit 2
    ;;
esac
