#!/bin/bash
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
#
# Board settings for the Arduino VENTUNO Q (Qualcomm Dragonwing IQ-8275 /
# QCS8300 family, Hexagon HTP v75, Ubuntu 24.04). Every launcher script
# sources this file, so board-specific paths live in one place.
#
#   source "$REPO/scripts/board-env.sh"
#
# Override any value by exporting it before running a script, e.g.
#   CONDA_BASE=/opt/miniforge3 ./start.sh ...

# Python env: miniforge in the board user's home (the VENTUNO Q runs as the
# `arduino` user, not root as on the Tria).
CONDA_BASE="${CONDA_BASE:-$HOME/miniforge3}"
CONDA_ENV="${CONDA_ENV:-iotc-ventuno-xarm}"
# shellcheck disable=SC2034  # used by the scripts that source this file
PYTHON="$CONDA_BASE/envs/$CONDA_ENV/bin/python3"

if [ -f "$CONDA_BASE/etc/profile.d/conda.sh" ]; then
    # shellcheck disable=SC1091
    source "$CONDA_BASE/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV"
else
    echo "[board_env] WARNING: $CONDA_BASE not found — run ./bootstrap-new-board.sh first" >&2
fi

# QNN HTP delegate runtime. Ubuntu's qairt-libs puts the delegate in /usr/lib
# (same path as the Tria); qairt-dsp-binaries links the Hexagon skel libs for
# this SoC into /usr/lib/dsp/cdsp (the Tria used /usr/lib/rfsa/adsp).
export ADSP_LIBRARY_PATH="${ADSP_LIBRARY_PATH:-/usr/lib/dsp/cdsp}"
export LD_LIBRARY_PATH="/usr/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# HDMI display: board_use_display points GUI apps at this user's GNOME
# desktop (X11 or Wayland). Returns 1 when nobody is logged in on HDMI, so
# SSH / boot-time launches fall back to headless + web stream.
_BOARD_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$_BOARD_ENV_DIR/../deploy/display-env.sh"
