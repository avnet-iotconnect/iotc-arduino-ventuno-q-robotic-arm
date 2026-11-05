# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Persistent, live-settable configuration for the YOLO grab gate.

Three settings, all changeable from IOTCONNECT with no mode restart (the
yolo modes re-read this store every frame):

  D_threshold   depth floor — gate satisfied when smoothed MiDaS D >= this
                (higher = closer). Default 750.
                Command: set_grab_threshold <value>

  gate_mode     which signals must agree before the grab fires:
                  'depth'  — MiDaS depth only
                  'radius' — apparent ball radius only (known-size ball:
                             r_px grows as the ball nears; far steadier
                             than INT8 MiDaS)
                  'both'   — grab needs depth AND radius; descent continues
                             while EITHER still reads far. Default.
                Command: set_grab_gate <depth|radius|both>

  radius_target radius floor in px — gate satisfied when ball r >=
                (radius_target - RADIUS_TOLERANCE). None = auto: use the
                taught ball_r_at_grab from grab_depth.json, falling back
                to TARGET_RADIUS_PX. Default auto.
                Command: set_grab_radius <px|auto>

Persisted to grab_threshold.json; `grab_threshold_reset` restores all
three defaults.

Teaching (save_taught): the calibrate_offset view-pose snapshot and the
calibrate grab_depth tool both record the ball's depth + YOLO radius at the
pose where the arm should decide to grab, write grab_depth.json, and set
D_threshold to D_grab minus a noise margin. The calibrators run as separate
processes, so the yolo modes call reload() when they start.
"""

import json
import os
import statistics
import time

DEFAULT_THRESHOLD = 750.0
DEFAULT_MODE = 'both'
GATE_MODES = ('depth', 'radius', 'both')
PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    'grab_threshold.json')
# Taught reference (D_grab, ball_r_at_grab) — read by modes/yolo_pickplace.py.
GRAB_DEPTH_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'grab_depth.json')
# Margin below the taught D for the depth floor: max(FLOOR, MULT * stdev).
DEPTH_TOL_FLOOR = 20.0
DEPTH_TOL_MULT = 1.5

_cfg = None


def _load():
    global _cfg
    _cfg = {'D_threshold': DEFAULT_THRESHOLD,
            'gate_mode': DEFAULT_MODE,
            'radius_target': None}
    if os.path.exists(PATH):
        try:
            with open(PATH) as f:
                data = json.load(f)
            v = float(data.get('D_threshold', DEFAULT_THRESHOLD))
            m = str(data.get('gate_mode', DEFAULT_MODE)).lower()
            r = data.get('radius_target')
            _cfg['D_threshold'] = v
            _cfg['gate_mode'] = m if m in GATE_MODES else DEFAULT_MODE
            _cfg['radius_target'] = float(r) if r is not None else None
        except (OSError, ValueError, TypeError, json.JSONDecodeError) as e:
            print(f"[grab_gate] load failed ({PATH}): {e} — using defaults")


def _save():
    with open(PATH, 'w') as f:
        json.dump(_cfg, f, indent=2)


def _ensure():
    if _cfg is None:
        _load()


def get():
    """Depth threshold. Cheap (cached) — safe to call every frame."""
    _ensure()
    return _cfg['D_threshold']


def get_mode():
    """Gate mode: 'depth' | 'radius' | 'both'."""
    _ensure()
    return _cfg['gate_mode']


def get_radius():
    """Explicit radius target in px, or None for auto (taught/constant)."""
    _ensure()
    return _cfg['radius_target']


def set_value(value):
    """Validate, persist, and apply a new depth threshold."""
    _ensure()
    v = float(value)
    if not 0 < v < 5000:
        raise ValueError(f"threshold {v:g} outside sane range 1-4999")
    _cfg['D_threshold'] = v
    _save()
    print(f"[grab_gate] depth floor set to D >= {v:g} (saved {PATH})")
    return v


def set_mode(mode):
    """Set which gate signals must agree ('depth'|'radius'|'both')."""
    _ensure()
    m = str(mode).strip().lower()
    if m not in GATE_MODES:
        raise ValueError(f"mode '{mode}' not one of {GATE_MODES}")
    _cfg['gate_mode'] = m
    _save()
    print(f"[grab_gate] gate mode set to '{m}' (saved {PATH})")
    return m


def set_radius(value):
    """Set explicit radius target in px, or None/'auto' for taught value."""
    _ensure()
    if value is None or str(value).strip().lower() in ('auto', 'none', 'reset'):
        _cfg['radius_target'] = None
        _save()
        print(f"[grab_gate] radius target set to auto (taught ball_r_at_grab)")
        return None
    v = float(value)
    if not 10 <= v <= 400:
        raise ValueError(f"radius {v:g} px outside sane range 10-400")
    _cfg['radius_target'] = v
    _save()
    print(f"[grab_gate] radius floor set to r >= {v:g} px (saved {PATH})")
    return v


def reset():
    """All three settings back to defaults; removes the override file."""
    global _cfg
    _cfg = None
    if os.path.exists(PATH):
        try:
            os.remove(PATH)
            print(f"[grab_gate] removed {PATH}")
        except OSError as e:
            print(f"[grab_gate] could not remove {PATH}: {e}")
    _ensure()
    return dict(_cfg)


def reload():
    """Re-read grab_threshold.json (another process — a calibrator — may
    have changed it since this one cached it)."""
    _load()


def save_taught(d_vals, r_vals, model='', depth_model=''):
    """Store a taught grab reference and set the depth floor from it.

    ``d_vals`` / ``r_vals``: per-frame depth at the ball and YOLO radius,
    captured while the arm is hand-held at the pose where it should decide
    to grab. Writes grab_depth.json and sets D_threshold = D_grab - margin.
    Returns (payload, threshold). Raises ValueError when the capture is
    unusable (too few frames, or zero spread = torque was on)."""
    if len(d_vals) < 5:
        raise ValueError(f"only {len(d_vals)} frames with ball + depth — keep the ball in view")
    D_grab = statistics.median(d_vals)
    r_grab = statistics.median(r_vals)
    stdev_d = statistics.stdev(d_vals)
    if stdev_d < 0.5:
        raise ValueError("depth didn't vary at all across frames — torque was ON during the "
                         "snapshot. Release torque, hand-hold the pose, snapshot again.")
    payload = {
        "D_grab": float(D_grab),
        "D_min": float(min(d_vals)),
        "D_max": float(max(d_vals)),
        "D_stdev": float(stdev_d),
        "ball_r_at_grab": float(r_grab),
        "n_frames": len(d_vals),
        "model": os.path.basename(model),
        "depth_model": os.path.basename(depth_model),
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
    }
    with open(GRAB_DEPTH_PATH, "w") as f:
        json.dump(payload, f, indent=2)
    print(f"[grab_gate] taught reference saved to {GRAB_DEPTH_PATH}: {payload}")
    margin = max(DEPTH_TOL_FLOOR, DEPTH_TOL_MULT * stdev_d)
    threshold = set_value(max(1.0, round(D_grab - margin, 1)))
    return payload, threshold


def describe():
    """One-line human summary for command acks and startup logs."""
    _ensure()
    r = _cfg['radius_target']
    return (f"mode={_cfg['gate_mode']}  D >= {_cfg['D_threshold']:g}  "
            f"r >= {f'{r:g}px' if r is not None else 'auto (taught)'}")
