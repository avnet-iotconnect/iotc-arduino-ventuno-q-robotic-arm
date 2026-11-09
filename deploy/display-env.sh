# shellcheck shell=sh
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
#
# board_use_display: point GUI apps (cv2.imshow) at the HDMI desktop of the
# current user. POSIX sh, so both board-env.sh (bash) and arm_watchdog.sh
# (sh) can source it.
#
# The VENTUNO Q desktop is GNOME, on X11 (DISPLAY=:1) by default, Wayland if
# switched. Instead of guessing, copy the display variables from the user's
# running gnome-shell. Returns 1 (and sets nothing) when nobody is logged in
# on HDMI, so callers fall back to headless + web stream.
board_use_display() {
    _pid=$(pgrep -u "$(id -u)" -x gnome-shell | head -n 1)
    [ -n "$_pid" ] || return 1
    _env=$(tr '\0' '\n' < "/proc/$_pid/environ" 2>/dev/null)
    _disp=$(printf '%s\n' "$_env" | sed -n 's/^DISPLAY=//p' | head -n 1)
    _wl=$(printf '%s\n' "$_env" | sed -n 's/^WAYLAND_DISPLAY=//p' | head -n 1)
    _xauth=$(printf '%s\n' "$_env" | sed -n 's/^XAUTHORITY=//p' | head -n 1)
    [ -n "$_disp$_wl" ] || return 1

    XDG_RUNTIME_DIR="/run/user/$(id -u)"
    export XDG_RUNTIME_DIR
    if [ -n "$_wl" ]; then
        export WAYLAND_DISPLAY="$_wl"
        export QT_QPA_PLATFORM=wayland
    else
        unset WAYLAND_DISPLAY
        export QT_QPA_PLATFORM=xcb
    fi
    [ -n "$_disp" ] && export DISPLAY="$_disp"
    [ -n "$_xauth" ] && export XAUTHORITY="$_xauth"
    export LANG=C.UTF-8 LC_ALL=C.UTF-8   # Qt wants a UTF-8 locale
    return 0
}
