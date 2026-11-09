# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Persistent camera tuning, applied identically across the demo + calibrators.

Settings live in `camera_settings.json` in the repo root. Anything that opens
a `cv2.VideoCapture` (main.py, browser_calibrate.py, browser_calibrate_offset.py)
loads the same file at startup so what you calibrate under is what you demo
under. Cloud commands `camera_setting`, `camera_settings_show`,
`camera_settings_reset` update the file at runtime; `camera_preset NAME`
replaces it wholesale with a named preset (built-in, or one you snapshot with
`camera_preset_save NAME` into `camera_presets.json`).

Why this matters: USB cameras default to auto-exposure + auto-white-balance,
which means today's HSV calibration won't match tomorrow's demo. Lock the
camera with `auto_exposure: 1`, `auto_wb: 0`, and a fixed `exposure` value,
then everything stays consistent.

Setting names map to V4L2 / OpenCV properties — see SETTING_PROPS below.
The bootstrap writes this default (Brio 100/101 on the VENTUNO Q):

  {
    "auto_wb":        0,
    "wb_temperature": "auto",  # measured for this room on first open, then
                               # locked and saved as a number (see apply())
    "auto_exposure":  1,       # 1 = manual on most V4L2 backends, 3 = auto
    "exposure":       250,
    "gain":           10,
    "saturation":     140
  }

A fixed white balance only suits one kind of light: 5400 K under warm indoor
LEDs turns the whole picture orange. "auto" avoids hard-coding it.
"""

from __future__ import annotations

import json
import os
import re

import cv2

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'camera_settings.json')

# Friendly setting name → (cv2 property constant, value caster).
# Cast keeps cloud clients from having to send the right Python type — they
# can send "−7" as a string and we'll float() it.
SETTING_PROPS = {
    'auto_exposure':  (cv2.CAP_PROP_AUTO_EXPOSURE, float),
    'exposure':       (cv2.CAP_PROP_EXPOSURE, float),
    'auto_wb':        (cv2.CAP_PROP_AUTO_WB, float),
    'wb_temperature': (cv2.CAP_PROP_WB_TEMPERATURE, float),
    'brightness':     (cv2.CAP_PROP_BRIGHTNESS, float),
    'contrast':       (cv2.CAP_PROP_CONTRAST, float),
    'saturation':     (cv2.CAP_PROP_SATURATION, float),
    'gain':           (cv2.CAP_PROP_GAIN, float),
    'sharpness':      (cv2.CAP_PROP_SHARPNESS, float),
    'gamma':          (cv2.CAP_PROP_GAMMA, float),
    'hue':            (cv2.CAP_PROP_HUE, float),
}


def load(path: str = DEFAULT_PATH) -> dict:
    """Read the settings JSON. Returns {} if the file doesn't exist or is bad."""
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError) as e:
        print(f"[camera_settings] load failed ({path}): {e}")
        return {}


def save(settings: dict, path: str = DEFAULT_PATH) -> None:
    try:
        with open(path, 'w') as f:
            json.dump(settings, f, indent=2)
        print(f"[camera_settings] saved to {path}: {settings}")
    except OSError as e:
        print(f"[camera_settings] save failed ({path}): {e}")


WB_AUTO = 'auto'
WB_FALLBACK_K = 4000          # used if the camera can't report its measurement
_WB_MEASURE_MAX_S = 6.0       # give auto white balance this long to settle


WB_SEARCH_MIN_K, WB_SEARCH_MAX_K = 2800, 7500   # Brio 100/101 manual range


def _red_blue(cap: cv2.VideoCapture, frames: int = 8) -> float | None:
    """log(red/blue) of the average colour, after letting a change settle.
    None if the camera returns no frames."""
    import math
    frame = None
    for _ in range(frames):
        ok, f = cap.read()
        if ok:
            frame = f
    if frame is None:
        return None
    b, _g, r = frame.reshape(-1, 3).mean(0)
    return math.log((r + 1.0) / (b + 1.0))


def _measure_wb(cap: cv2.VideoCapture) -> int | None:
    """Return the manual white balance (K) that matches what the camera's
    auto white balance picks for this room, or None if it can't be measured.

    The temperature a UVC camera *reports* while in auto mode is on a
    different scale from its manual setting (a Brio 101 reported 5340 K for
    a picture that manual 4600 K reproduces; locking 5340 K came out deep
    orange). So: let auto settle, record the picture's red/blue balance,
    then binary-search the manual setting that gives the same balance.
    Higher manual K = warmer picture, which makes the search well-defined."""
    import time
    cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    start, target, prev = time.monotonic(), None, None
    while time.monotonic() - start < _WB_MEASURE_MAX_S:
        rb = _red_blue(cap, frames=10)
        if rb is None:
            return None
        if prev is not None and abs(rb - prev) < 0.02 and time.monotonic() - start > 1.5:
            target = rb
            break
        prev = rb
    if target is None:
        target = prev
    cap.set(cv2.CAP_PROP_AUTO_WB, 0)
    lo, hi = WB_SEARCH_MIN_K, WB_SEARCH_MAX_K
    while hi - lo > 50:
        mid = (lo + hi) // 2
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, mid)
        rb = _red_blue(cap)
        if rb is None:
            return None
        if rb > target:
            hi = mid        # too warm -> go lower
        else:
            lo = mid
    return int(round((lo + hi) / 2 / 10.0)) * 10


def _resolve_auto_wb(cap: cv2.VideoCapture, settings: dict, path: str) -> dict:
    """`wb_temperature: "auto"` means: measure this room's white balance once,
    lock it, and save the number. Every later open (demo, calibrators) then
    uses the same fixed value, so colours calibrated under it stay put. Send
    `camera_setting wb_temperature auto` to re-measure at a new venue."""
    measured = _measure_wb(cap)
    k = measured or WB_FALLBACK_K
    print(f"[camera_settings] auto white balance measured "
          f"{measured if measured else 'nothing'} -> locking {k} K")
    settings = dict(settings, auto_wb=0, wb_temperature=k)
    # Save so later opens reuse this value rather than measure again. Only
    # the WB keys change; anything else in the file is left as it was.
    on_disk = load(path)
    if str(on_disk.get('wb_temperature', '')).lower() == WB_AUTO:
        on_disk.update(auto_wb=0, wb_temperature=k)
        save(on_disk, path)
    return settings


def apply(cap: cv2.VideoCapture, settings: dict, path: str | None = None) -> None:
    """Apply each setting from ``settings`` to an opened cv2.VideoCapture.
    Order matters for some V4L2 stacks: switch auto_* off first, then set the
    fixed values, otherwise the camera reverts. We use a fixed sort order to
    enforce that.

    ``wb_temperature: "auto"`` is measured first and replaced by the
    measured value, which is saved to ``path`` (default camera_settings.json)."""
    if not settings:
        return
    if str(settings.get('wb_temperature', '')).lower() == WB_AUTO:
        # Measure under the final exposure / gain / saturation.
        apply(cap, {k: v for k, v in settings.items()
                    if k not in ('auto_wb', 'wb_temperature')})
        settings = _resolve_auto_wb(cap, settings, path or DEFAULT_PATH)
    # auto_* keys go first so the corresponding fixed values stick.
    ordered_keys = sorted(settings.keys(),
                          key=lambda k: (0 if k.startswith('auto_') else 1, k))
    for name in ordered_keys:
        value = settings[name]
        prop = SETTING_PROPS.get(name)
        if prop is None:
            print(f"[camera_settings] unknown setting '{name}' — skipping")
            continue
        prop_id, cast = prop
        try:
            ok = cap.set(prop_id, cast(value))
            actual = cap.get(prop_id)
            print(f"[camera_settings] {name}={value} -> set_ok={ok}  read_back={actual}")
        except Exception as e:
            print(f"[camera_settings] failed to set {name}={value}: {e}")


def update_one(name: str, value: float | str, path: str = DEFAULT_PATH) -> dict | None:
    """Load → set one key → save. Returns the updated dict (or None on bad name)."""
    if name not in SETTING_PROPS:
        return None
    settings = load(path)
    settings[name] = value
    save(settings, path)
    return settings


def reset(path: str = DEFAULT_PATH) -> dict:
    """Wipe to empty (camera reverts to its V4L2 defaults next open)."""
    save({}, path)
    return {}


def known_setting_names() -> list[str]:
    return sorted(SETTING_PROPS.keys())


# ---- presets ---------------------------------------------------------------
# A preset is a COMPLETE settings dict, not a delta: `camera_preset bright`
# replaces camera_settings.json outright, so the picture you get doesn't depend
# on which buttons were pressed before. Every preset therefore carries the same
# base keys; anything a preset doesn't list keeps the camera's last value.
#
# Built-in values are for the Innomaker U20CAM (exposure in 0.1 ms steps,
# gain 0-100, saturation 0-128, wb_temperature 2800-6500 K). For a different
# camera, tune with `camera_setting` and snapshot with `camera_preset_save`.
PRESETS_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                            'camera_presets.json')

_BASE = {
    'auto_exposure': 1,   # manual — no drift when someone walks past
    'exposure': 70,
    'gain': 6,
    'auto_wb': 1,
    'saturation': 95,
    'contrast': 32,
    'brightness': 0,
    'gamma': 100,
}

BUILTIN_PRESETS = {
    'normal': dict(_BASE),
    'bright': dict(_BASE, exposure=45, gain=0),                  # bright room / white-table glare
    'dim':    dict(_BASE, exposure=110, gain=15, saturation=100),  # evening / dim venue
    'vivid':  dict(_BASE, saturation=120, contrast=48),          # ball colour dull, edges soft
    'warm':   dict(_BASE, auto_wb=0, wb_temperature=3500),       # tungsten lights → kill yellow cast
    'cool':   dict(_BASE, auto_wb=0, wb_temperature=5500),       # daylight / cool LED → kill blue cast
    # Let the camera meter the room; exposure is omitted because the driver
    # rejects a manual exposure while auto is on.
    'auto':   {k: v for k, v in dict(_BASE, auto_exposure=3).items() if k != 'exposure'},
}

_PRESET_NAME_OK = re.compile(r'^[a-z0-9][a-z0-9_-]{0,31}$')


def _load_user_presets(path: str = PRESETS_PATH) -> dict[str, dict]:
    if not os.path.exists(path):
        return {}
    try:
        with open(path) as f:
            data = json.load(f)
        return {str(k).lower(): v for k, v in data.items() if isinstance(v, dict)}
    except (OSError, json.JSONDecodeError, AttributeError) as e:
        print(f"[camera_settings] presets load failed ({path}): {e}")
        return {}


def load_presets(path: str = PRESETS_PATH) -> dict[str, dict]:
    """Built-in presets overlaid with any saved in camera_presets.json
    (a saved preset with the same name as a built-in wins)."""
    presets = {k: dict(v) for k, v in BUILTIN_PRESETS.items()}
    presets.update(_load_user_presets(path))
    return presets


def preset_names(path: str = PRESETS_PATH) -> list[str]:
    return sorted(load_presets(path).keys())


def apply_preset(name: str, path: str = DEFAULT_PATH, presets_path: str = PRESETS_PATH) -> dict | None:
    """Replace camera_settings.json with the named preset. Returns the new
    settings dict, or None if the name is unknown. Caller should mark_dirty()
    so the live capture thread re-applies it."""
    preset = load_presets(presets_path).get(str(name or '').strip().lower())
    if preset is None:
        return None
    settings = {k: v for k, v in preset.items() if k in SETTING_PROPS}
    save(settings, path)
    return settings


def save_preset(name: str, path: str = DEFAULT_PATH, presets_path: str = PRESETS_PATH) -> dict | None:
    """Snapshot the current camera_settings.json under NAME in
    camera_presets.json, so a picture tuned with `camera_setting` becomes a
    one-press `camera_preset NAME`. Returns the saved dict, or None if the
    name is invalid or there are no current settings to save."""
    name = str(name or '').strip().lower()
    settings = load(path)
    if not _PRESET_NAME_OK.match(name) or not settings:
        return None
    user = _load_user_presets(presets_path)
    user[name] = settings
    try:
        with open(presets_path, 'w') as f:
            json.dump(user, f, indent=2)
        print(f"[camera_settings] preset '{name}' saved to {presets_path}: {settings}")
    except OSError as e:
        print(f"[camera_settings] preset save failed ({presets_path}): {e}")
        return None
    return settings


# ---- runtime re-apply signaling -------------------------------------------
# When a cloud command updates camera_settings.json, the live VideoCapture
# (owned by main.py's _FreshCamera) needs to pick up the new values without
# restarting. The capture loop polls is_dirty() every iteration; when set,
# it reloads the JSON and re-applies it.
_dirty = False


def mark_dirty() -> None:
    global _dirty
    _dirty = True


def is_dirty() -> bool:
    return _dirty


def clear_dirty() -> None:
    global _dirty
    _dirty = False


def find_brio_index(fallback: int = 2) -> int:
    """Return the /dev/videoN index of the Brio's capture node.

    The Brio's V4L2 enumeration drifts across USB events (we've seen it on
    /dev/video2 and /dev/video3 in the same week), so hard-coding --camera
    eventually breaks. The Linux kernel labels each /dev/videoN with the
    camera's model string at ``/sys/class/video4linux/videoN/name``; the
    Brio prints "Brio 100" (or similar) there. We pick the lowest matching
    index — when the Brio enumerates as a pair, the lower-numbered node is
    the capture node (full V4L2 controls); the sibling is a metadata node
    that ignores cap.set().

    Returns ``fallback`` if no Brio node is found, so behaviour on a
    non-Brio rig (or boot without the camera plugged in) is the same as
    before this helper existed."""
    import glob, os
    usb_nodes = []
    named_nodes = []
    for path in sorted(glob.glob("/sys/class/video4linux/video*")):
        try:
            idx = int(os.path.basename(path).replace("video", ""))
        except ValueError:
            continue
        # A USB webcam's `device` symlink resolves under a .../usb.../ path,
        # while the Qualcomm platform/codec nodes (video0/1/32/33) do not.
        # This finds ANY USB camera (Brio, Innomaker U20CAM, eMeet, ...), not
        # just the Brio. A UVC camera enumerates as a node pair; the lower
        # index is the capture node, the sibling is a metadata node.
        try:
            if "usb" in os.path.realpath(os.path.join(path, "device")):
                usb_nodes.append(idx)
        except OSError:
            pass
        # Secondary cross-check: match known webcam model strings by name.
        try:
            with open(os.path.join(path, "name")) as f:
                nm = f.read().strip().lower()
            if any(t in nm for t in ("brio", "innomaker", "u20cam", "webcam",
                                     "emeet", "vitade", "microdia", "uvc")):
                named_nodes.append(idx)
        except OSError:
            pass
    both = sorted(set(usb_nodes) & set(named_nodes))
    if both:
        return both[0]
    if usb_nodes:
        return min(usb_nodes)
    if named_nodes:
        return min(named_nodes)
    print(f"[camera] WARNING: no USB camera found in /sys/class/video4linux — "
          f"falling back to /dev/video{fallback}. "
          f"If that fails, run: v4l2-ctl --list-devices  then pass --camera N")
    return fallback
