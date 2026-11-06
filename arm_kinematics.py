# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Planar kinematics for the xArm 1S shoulder -> elbow -> wrist chain.

Used to replay the taught grab reach (calibrate_offset: view pose -> jaws
around the ball) as a move of the *gripper*, not as raw joint deltas.

Why: the grab fires wherever tracking ends, and that is rarely the exact
arm shape it was taught in (a ball nearer or farther than at calibration
needs a different elbow / wrist). The same joint deltas from a different
shape move the gripper somewhere else; on a VENTUNO Q build, an elbow 25
degrees more bent left the jaws ~2 cm short of the ball. The ball sits in
the same place relative to the gripper either way (same pixel, same size
in the wrist camera), so the right move is the same one relative to the
gripper.

The model: shoulder_lift, elbow_flex and wrist_flex all turn in one
vertical plane. Angles are measured from straight up (all servos at 500),
positive = leaning toward the front of the arm. Only the two link lengths
up to the wrist axis matter; the gripper is one rigid body after it, so its
length cancels. The lengths are approximate; the reach is a few cm, so
small errors in them change the result by millimetres.
"""

import math

DEG_PER_UNIT = 0.24          # bus servo: 1000 units = 240 degrees
UPPER_ARM_MM = 100.0         # shoulder_lift axis -> elbow_flex axis (approx.)
FOREARM_MM = 100.0           # elbow_flex axis -> wrist_flex axis (approx.)

SERVO_WRIST_FLEX = 3
SERVO_ELBOW_FLEX = 4
SERVO_SHOULDER_LIFT = 5
CHAIN = (SERVO_SHOULDER_LIFT, SERVO_ELBOW_FLEX, SERVO_WRIST_FLEX)


def joint_signs(mirrored):
    """+1/-1 per (lift, elbow, wrist): servo units -> forward-positive angle.

    The Tria's arm and its mirror-image build turn every joint the opposite
    way. Within one build, the wrist and shoulder count the opposite way to
    the elbow (on the mirror-image build: forward lean = lift down, elbow
    up, wrist down; checked against a taught grab, which keeps the gripper's
    tilt within a few degrees as it should)."""
    return (-1, 1, -1) if mirrored else (1, -1, 1)


def _angles(pose, signs):
    return [math.radians(s * (int(pose[sid]) - 500) * DEG_PER_UNIT)
            for sid, s in zip(CHAIN, signs)]


def forward(pose, signs):
    """(x, z, phi): wrist axis position (mm, from the shoulder axis) and the
    gripper direction (radians from straight up)."""
    a1, a2, a3 = _angles(pose, signs)
    x = UPPER_ARM_MM * math.sin(a1) + FOREARM_MM * math.sin(a1 + a2)
    z = UPPER_ARM_MM * math.cos(a1) + FOREARM_MM * math.cos(a1 + a2)
    return x, z, a1 + a2 + a3


def inverse(x, z, phi, signs, elbow_like):
    """Servo positions {servo: units} that put the wrist axis at (x, z) with
    the gripper at angle phi, or None if out of reach / out of servo range.
    ``elbow_like`` (an elbow angle, radians) picks the elbow branch."""
    L1, L2 = UPPER_ARM_MM, FOREARM_MM
    c2 = (x * x + z * z - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1.0:
        return None
    a2 = math.acos(c2) * (1 if elbow_like >= 0 else -1)
    a1 = math.atan2(x, z) - math.atan2(L2 * math.sin(a2), L1 + L2 * math.cos(a2))
    a3 = phi - a1 - a2
    out = {}
    for sid, s, a in zip(CHAIN, signs, (a1, a2, a3)):
        units = 500 + math.degrees(a) / DEG_PER_UNIT * s
        if not 0 <= units <= 1000:
            return None
        out[sid] = int(round(units))
    return out


def gripper_frame_reach(view_pose, reach, here, signs):
    """Replay the taught reach (view_pose -> view_pose + reach) as the same
    move relative to the gripper, starting from ``here``.

    view_pose / here: {servo: units} including lift, elbow and wrist.
    reach: {servo: delta units}. Returns {servo: units} for lift, elbow and
    wrist, or None if the move can't be solved (caller falls back to raw
    joint deltas)."""
    try:
        view = {sid: int(view_pose[sid]) for sid in CHAIN}
        grab = {sid: view[sid] + int(reach.get(sid, 0)) for sid in CHAIN}
        now = {sid: int(here[sid]) for sid in CHAIN}
    except (KeyError, TypeError, ValueError):
        return None
    xv, zv, pv = forward(view, signs)
    xg, zg, pg = forward(grab, signs)
    # Taught move in the gripper's own frame: along it (u) and across it (n).
    dx, dz = xg - xv, zg - zv
    du = dx * math.sin(pv) + dz * math.cos(pv)
    dn = dx * math.cos(pv) - dz * math.sin(pv)
    xn, zn, pn = forward(now, signs)
    xt = xn + du * math.sin(pn) + dn * math.cos(pn)
    zt = zn + du * math.cos(pn) - dn * math.sin(pn)
    return inverse(xt, zt, pn + (pg - pv), signs, _angles(now, signs)[1])
