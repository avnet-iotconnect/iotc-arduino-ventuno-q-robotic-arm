# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Persistent camera-gripper offset + ball-grab radius.

Without calibration, the controller assumes the camera optical axis lines up
exactly with the gripper's grab point — almost no real wrist mount achieves
that, so when the camera says "ball is centered," the gripper is a few cm
off and grabs air. The offset calibrator measures the offset (pose the
gripper over the ball, snapshot, get pixel offset between ball center and
image center) and writes the result here. ball + pickplace load it at
mode-instance construction.

JSON format:
{
  "cam_gripper_offset_x": -8,    # ball pixel X − image center X, at the view pose
  "cam_gripper_offset_y": 14,    # ball pixel Y − image center Y
  "target_radius_px":   220,     # ball's apparent radius at the view pose
  "view_pose":  {"3": 870, "4": 300, "5": 420, "6": 510},   # optional
  "grab_reach": {"3": 40, "4": 85, "5": 60, "6": 0}          # optional
}

Two-step calibration (browser_calibrate_offset.py):

  1. VIEW pose — the gripper just above/behind the ball with the ball fully
     in camera view. Records the offset + radius above, and the joint pose.
     This is where tracking stops.
  2. GRAB pose (optional) — the gripper around the ball, ball not necessarily
     visible. Records ``grab_reach``: per-servo deltas (wrist flex, elbow,
     shoulder lift, shoulder pan) from the view pose to the grab pose. At
     grab time the arm makes that move without the camera, then closes.

Without step 2 (no ``grab_reach``) the gripper closes where tracking stops —
the original behaviour, for mounts where the ball stays visible between
the jaws. Re-doing step 1 clears the reach, since it is relative to the
view pose.

Falls back to (0, 0, modes/ball_follow.TARGET_RADIUS_PX, no reach) when the
file is absent — same as the pre-calibration behaviour.
"""

import json
import os

DEFAULT_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            'cam_gripper_offset.json')


def load(path=DEFAULT_PATH):
    """Return the stored dict or None if no file / unreadable."""
    if not os.path.exists(path):
        return None
    try:
        with open(path) as f:
            data = json.load(f)
        if not isinstance(data, dict):
            return None
        return data
    except (OSError, json.JSONDecodeError) as e:
        print(f"[cam_gripper_offset] load failed ({path}): {e}")
        return None


# Servos the grab reach moves: wrist flex, elbow, shoulder lift, shoulder pan.
# Not the gripper (the grab opens/closes it) or wrist roll (tracking holds it
# neutral; hand-posing would only add noise).
REACH_SERVOS = (3, 4, 5, 6)


def _write(payload, path):
    try:
        with open(path, 'w') as f:
            json.dump(payload, f, indent=2)
        print(f"[cam_gripper_offset] saved to {path}: {payload}")
        return payload
    except OSError as e:
        print(f"[cam_gripper_offset] save failed ({path}): {e}")
        return None


def save(off_x, off_y, target_radius_px, path=DEFAULT_PATH, view_pose=None):
    """Step 1: save the offset + radius seen at the view pose. ``view_pose``
    ({servo_id: pos}) is stored so step 2 can compute the reach. Clears any
    previous grab reach (it was relative to the old view pose)."""
    payload = {
        'cam_gripper_offset_x': int(off_x),
        'cam_gripper_offset_y': int(off_y),
        'target_radius_px': int(target_radius_px),
    }
    if view_pose:
        payload['view_pose'] = {str(s): int(view_pose[s]) for s in REACH_SERVOS}
    return _write(payload, path)


def save_reach(grab_pose, path=DEFAULT_PATH):
    """Step 2: store the move from the saved view pose to ``grab_pose``
    ({servo_id: pos}). Returns the saved dict, or None if step 1 (with a
    view pose) hasn't been done yet."""
    data = load(path)
    if not data or not data.get('view_pose'):
        return None
    view = data['view_pose']
    data['grab_reach'] = {str(s): int(grab_pose[s]) - int(view[str(s)]) for s in REACH_SERVOS}
    return _write(data, path)


def clear_reach(path=DEFAULT_PATH):
    """Drop the grab reach (back to closing where tracking stops)."""
    data = load(path)
    if data and 'grab_reach' in data:
        del data['grab_reach']
        return _write(data, path)
    return data


def load_reach(path=DEFAULT_PATH):
    """Return the grab reach as {servo_id(int): delta} or None if not taught."""
    data = load(path)
    reach = (data or {}).get('grab_reach')
    if not isinstance(reach, dict):
        return None
    try:
        return {int(s): int(d) for s, d in reach.items()}
    except (TypeError, ValueError):
        return None


def reset(path=DEFAULT_PATH):
    if os.path.exists(path):
        try:
            os.remove(path)
            print(f"[cam_gripper_offset] removed {path}")
        except OSError as e:
            print(f"[cam_gripper_offset] could not remove {path}: {e}")


def show(path=DEFAULT_PATH):
    return load(path) or {}
