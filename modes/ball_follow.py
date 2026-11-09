# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Ball-follow mode (eye-in-hand visual servoing).

Reads HSV thresholds from ball_color.json, segments the ball each frame, and
runs a small proportional controller to:
  1) center the ball in the wrist-camera frame (shoulder_pan + wrist_flex)
  2) advance until the ball's apparent radius hits a target (shoulder_lift)
  3) close the gripper, lift, return to home

State machine: IDLE -> TRACKING -> GRABBING -> AFTER_GRAB -> IDLE.
"""

import json
import os
import time
from collections import deque

import cv2
import numpy as np
from xarm import Servo

from core import arm_kinematics

from .base import Mode

CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "ball_color.json")

# --- tuning knobs ---
UPDATE_EVERY = 1           # send a servo command every N frames
PAN_GAIN = 0.04            # servo units per pixel of horizontal error
TILT_GAIN = 0.10           # higher than PAN_GAIN because wrist_flex fights gravity at extended poses
# Correction signs, found by live test. Flip one if the arm moves away from the ball on that axis.
# If wrist_flex is pinned near 1000 the tilt sign is not the problem: re-teach the scan pose with wrist_flex <= ~900.
PAN_DIR = 1
TILT_DIR = -1
# Bus servos ignore commands below a friction floor, so steps and trims are floored. The floor is higher at
# extended poses (shoulder_lift > 600) and on shoulder_pan, which carries the whole forearm + wrist + camera.
APPROACH_STEP = 15         # shoulder_lift step toward the ball
ELBOW_REACH_RATIO = 0.6    # elbow_flex contribution per unit of shoulder_lift during approach
TILT_ELBOW_RATIO = 0.0     # elbow assist for tilt; off because it drained elbow_flex to its limit over many frames
TILT_ELBOW_DIR = 1         # flip to -1 if elbow moves wrong way for tilt
MAX_STEP = 25              # cap on any single-frame servo change
MIN_TRIM_STEP = 14         # smallest wrist_flex correction (8 didn't move the loaded wrist)
MIN_TRIM_STEP_PAN = 18     # smallest shoulder_pan correction
MOVE_DURATION_MS = 220     # per-step move duration; under ~150 ms many bus servos ignore small moves
# One floored pan step shifts the image ~126 px (~7 px/unit), so a narrow left/right deadband makes pan overshoot
# back and forth. 60 px is still inside the jaw opening.
CENTER_DEADBAND_PX = 60
# Reaching shifts the ball in the image, so the ball must be this centered before reaching. If the wrist stalls just
# outside it, raise MIN_TRIM_STEP rather than widening this.
APPROACH_DEADBAND_PX = 60
TARGET_RADIUS_PX = 220     # fallback grab radius; normally replaced by calibrate_offset's target_radius_px
RADIUS_TOLERANCE = 40      # +/- around TARGET_RADIUS_PX considered "at distance" (legacy fixed value; see radius_tolerance())
RADIUS_TOLERANCE_FRAC = 0.15  # tolerance as a fraction of the target radius: a fixed 40 px suited
                              # the Tria's ball at r=220 but accepts r=28..108 for a ball taught at 68
MIN_CONTOUR_AREA = 200     # ignore tiny mask blobs (noise)
MIN_FILL_RATIO = 0.65      # contour_area / enclosing_circle_area; ball ≈0.85+, irregular blobs typically <0.5
# Fallback aim point offset from the image center, in pixels (raw, unmirrored frame). Normally replaced by the
# calibrate_offset view pose, so "centered" means "gripper over the ball" for this camera mount.
CAM_GRIPPER_OFFSET_X = 0
CAM_GRIPPER_OFFSET_Y = 0
# --- search envelope ---
# Minimum shoulder_pan range while tracking. pan_limits() widens it to cover every scan pose, otherwise a ball found
# from an edge scan pose can't be followed.
PAN_MIN = 90
PAN_MAX = 860
# Hard safety limits so an approach that never satisfies radius_ok (bad HSV, wrong-sized ball,
# mis-set TARGET_RADIUS_PX) can't drive the gripper into the table. Tune by teach-mode probing.
LIFT_MAX = 800             # shoulder_lift ceiling during approach (larger = reach further down)
ELBOW_MAX = 500            # elbow_flex ceiling during approach
# Defaults only, for an arm built like the Tria's. Arm builds differ (some
# are mirror images: elbow and wrist bend the other way), so modes work out
# the approach direction and limits per arm with ApproachPlan, from the
# calibrate_offset grab reach and the scan poses. A limit never moves a joint
# backwards, only stops it.
APPROACH_REACH = 250       # how far past the calibrated view pose the shoulder may reach
REACH_MARGIN = 100         # ... and the elbow, when reaching means bending it further
SCAN_DWELL_S = 1.5         # hold each scan pose this long before advancing
SCAN_MOVE_MS = 2500        # duration for the move between scan poses (slowed so right-pose lift=635 doesn't plunge toward table)
# Frames without a ball before scanning resumes (~5 s at 6 fps). Holding the pose through brief detection drop-outs
# stops the arm shaking between tracking and scanning.
NO_BALL_GRACE_FRAMES = 30
# --- prediction + telemetry ---
ENABLE_PREDICTION = False        # master switch for extrapolating ball motion when it's lost
POS_BUFFER_LEN = 5               # how many recent ball positions we keep for velocity est
MIN_VELOCITY_PX_PER_FRAME = 2.0  # velocity below this = don't bother predicting
MAX_PREDICT_FRAMES = 15          # cap predictions so the arm doesn't run away on a bad estimate
TELEMETRY_INTERVAL_S = 5.0       # how often we publish ball telemetry to /IOTCONNECT — matches device template dataFrequency
GRIPPER_CLOSE_TARGET = 1000  # commanded close position; actual may stall below this on large objects
GRIPPER_STALL_SLACK = 10    # if actual < target - this, assume stalled against object
GRIPPER_RELEASE_DELTA = 40  # if current pos < hold_target - this, user opened the gripper
GRIPPER_CLOSE_MS = 250          # close ramp; short so the servo is already at full torque when it meets the ball
GRIPPER_SETTLE_TIMEOUT_S = 3.0  # max extra time to wait for the jaws to stop moving (they creep for ~2 s on the ball)
GRIPPER_SETTLE_DELTA = 3        # consecutive reads this close together = jaws stopped
GRIPPER_SETTLE_READS = 3        # how many agreeing reads (100 ms apart) before we call the jaws stopped
GRIPPER_SETTLE_DWELL_S = 0.5    # let the servo build full torque on the ball before lifting
GRIPPER_EMPTY_POS = 900         # jaws at/after this after the lift = nothing in them (a ball holds them well below)
GRIPPER_TRACK_POS = 250         # jaw opening while tracking (matches the scan poses)
# Jaw tips with suction cups stick together when they close on nothing, and
# a plain "open" can stall against the seal. open_gripper_firmly() checks the
# jaws really opened and, if not, snaps them: a short squeeze, then open at
# full speed, a few times.
GRIPPER_UNSTICK_TRIES = 4
GRIPPER_OPEN_SLACK = 120        # opened = jaws within this of the commanded position
GRIPPER_SNAP_MS = 120           # fast open: full speed, so the jolt breaks the seal
GRIPPER_SQUEEZE = 60            # squeeze this much before each snap open
# Up/down tolerance before a grab. Left/right keeps CENTER_DEADBAND_PX (a
# single pan step moves the image ~126 px), but one wrist step only turns the
# camera ~3.4 deg (~32 px at any distance), so up/down can be held tighter.
# With the camera looking down and forward, a ball still HIGH in the image is
# farther away than taught and the jaws close short of it.
GRAB_TILT_DEADBAND_PX = 30
GRAB_REACH_MS = 1200            # duration of the taught blind move from the view pose to the grab pose

# --- arm conventions (from main.execute_arm_action) ---
SERVO_GRIPPER = 1
SERVO_WRIST_ROLL = 2
SERVO_WRIST_FLEX = 3       # smaller = camera tilts up; larger = camera tilts down
SERVO_ELBOW_FLEX = 4
SERVO_SHOULDER_LIFT = 5    # smaller = lift up; larger = reach forward/down
SERVO_SHOULDER_PAN = 6     # smaller = pan left; larger = pan right (verify w/ live test)

# Default scan poses, cycled through while no ball is seen (taught with teach_pose.py
# for a wall-mounted arm looking down at the table). wrist_roll forced to 500 (neutral) so the wrist_flex axis produces pure
# pitch. Off-neutral roll rotates the image, which decouples pixel dx/dy
# from the pan/flex servos and causes tracking to oscillate.
SCAN_POSES = [
    # center
    [[SERVO_SHOULDER_PAN, 499], [SERVO_SHOULDER_LIFT, 242], [SERVO_ELBOW_FLEX, 277],
     [SERVO_WRIST_FLEX, 900], [SERVO_WRIST_ROLL, 500], [SERVO_GRIPPER, 250]],
    # left edge
    [[SERVO_SHOULDER_PAN, 346], [SERVO_SHOULDER_LIFT, 213], [SERVO_ELBOW_FLEX, 241],
     [SERVO_WRIST_FLEX, 876], [SERVO_WRIST_ROLL, 494], [SERVO_GRIPPER, 258]],
    # right edge
    [[SERVO_SHOULDER_PAN, 736], [SERVO_SHOULDER_LIFT, 214], [SERVO_ELBOW_FLEX, 240],
     [SERVO_WRIST_FLEX, 855], [SERVO_WRIST_ROLL, 495], [SERVO_GRIPPER, 257]],
]
SCAN_POSE_LABELS = ["center", "left", "right"]

# TODO: replace with recaptured home pose — prior snapshot had a corrupt elbow reading.
HOME_POSE = [[s, 500] for s in range(1, 7)]
# Same as HOME_POSE but excludes the gripper, so a held object isn't dropped.
HOME_POSE_KEEP_GRIP = [[s, 500] for s in range(2, 7)]


# Reusable Servo objects so batched getPosition doesn't allocate per frame.
_ALL_SERVOS = [Servo(i) for i in range(1, 7)]


def _read_all_positions(arm):
    """Per-servo reads of all six positions. Returns {id: pos}.

    We do NOT use the batched ``arm.getPosition([Servo...])`` call: when its
    response parse throws IndexError it leaves leftover bytes in the HID read
    buffer, permanently desynchronising the stream. Every subsequent read
    then mis-parses and the process must be restarted with a USB power-cycle.
    Per-servo reads cost ~6 USB RTTs (~40 ms) but don't poison the stream.
    """
    return {sid: int(arm.getPosition(sid)) for sid in range(1, 7)}


def pan_limits(scan_poses):
    """Tracking clamp for shoulder_pan: [PAN_MIN, PAN_MAX], widened to cover
    every scan pose's pan. Otherwise a ball found from a scan pose outside the
    clamp can't be followed — pan pins at the limit and the grab never fires.
    Scan poses vary per build (taught with teach_scan_pose), so derive this
    instead of asking everyone to keep PAN_MIN/PAN_MAX in sync by hand."""
    pans = [int(p) for pose in scan_poses for sid, p in pose if sid == SERVO_SHOULDER_PAN]
    return min([PAN_MIN] + pans), max([PAN_MAX] + pans)


def _sign(v):
    return 1 if v > 0 else -1


class ApproachPlan:
    """Which way is 'toward the ball' for THIS arm, and how far to go.

    The approach steps shoulder_lift and elbow_flex together. Which sign
    reaches forward depends on how the arm was built: on the Tria's arm
    (elbow ~277 in the scan pose) both go up; on a mirror-image build
    (elbow ~845) both go down, and going up folds the arm back on itself.

    Direction, best source first:
      1. the calibrate_offset grab reach (view pose -> jaws around the
         ball): its shoulder/elbow move IS "toward the ball" for this arm;
      2. otherwise the scan pose: elbow above 500 = mirror-image build.
    Limits: the elbow stops at straight (500) when it is straightening,
    the shoulder APPROACH_REACH past the view pose (LIFT_MAX without one).
    When one joint runs out of room the other keeps going alone."""

    STRAIGHT = 500

    def __init__(self, scan_poses, view_pose=None, grab_reach=None):
        lifts = [int(p) for pose in scan_poses for sid, p in pose if sid == SERVO_SHOULDER_LIFT]
        elbows = [int(p) for pose in scan_poses for sid, p in pose if sid == SERVO_ELBOW_FLEX]
        scan_elbow = elbows[0] if elbows else 300
        self.mirrored = scan_elbow > self.STRAIGHT   # elbow bends the other way to the Tria's
        reach = grab_reach or {}
        r_lift = int(reach.get(SERVO_SHOULDER_LIFT, reach.get(str(SERVO_SHOULDER_LIFT), 0)) or 0)
        r_elbow = int(reach.get(SERVO_ELBOW_FLEX, reach.get(str(SERVO_ELBOW_FLEX), 0)) or 0)
        if abs(r_lift) >= 10:
            self.lift_dir = _sign(r_lift)
            ratio = r_elbow / abs(r_lift)
            self.elbow_dir = _sign(r_elbow) if r_elbow else _sign(self.STRAIGHT - scan_elbow)
            self.elbow_ratio = min(1.5, abs(ratio)) if r_elbow else ELBOW_REACH_RATIO
            self.source = f"taught grab reach (lift {r_lift:+d}, elbow {r_elbow:+d})"
        else:
            mirrored = scan_elbow > self.STRAIGHT
            self.lift_dir = -1 if mirrored else 1
            self.elbow_dir = -1 if mirrored else 1
            self.elbow_ratio = ELBOW_REACH_RATIO
            self.source = (f"scan pose (elbow {scan_elbow}: "
                           f"{'mirror-image' if mirrored else 'Tria-style'} build)")

        vp = view_pose or {}
        v_lift = vp.get(str(SERVO_SHOULDER_LIFT), vp.get(SERVO_SHOULDER_LIFT))
        v_elbow = vp.get(str(SERVO_ELBOW_FLEX), vp.get(SERVO_ELBOW_FLEX))
        if v_lift is not None:
            lim = int(v_lift) + self.lift_dir * APPROACH_REACH
        else:
            lim = LIFT_MAX if self.lift_dir > 0 else 1000 - LIFT_MAX
        # Never stricter than the scan poses, so the arm isn't 'blocked' before it moves.
        if lifts:
            lim = max([lim] + lifts) if self.lift_dir > 0 else min([lim] + lifts)
        self.lift_limit = max(0, min(1000, lim))

        e_ref = int(v_elbow) if v_elbow is not None else scan_elbow
        if (e_ref - self.STRAIGHT) * self.elbow_dir < 0:
            self.elbow_limit = self.STRAIGHT          # straightening: stop at straight
        else:
            self.elbow_limit = max(0, min(1000, e_ref + self.elbow_dir * REACH_MARGIN))

    @staticmethod
    def _room(pos, limit, d):
        return (limit - pos) * d > 0

    def step(self, lift, elbow):
        """(d_lift, d_elbow) for one approach step from the current pose, or
        None when both joints are at their limits (ball out of reach)."""
        lift_ok = self._room(lift, self.lift_limit, self.lift_dir)
        elbow_ok = self.elbow_ratio > 0 and self._room(elbow, self.elbow_limit, self.elbow_dir)
        if not lift_ok and not elbow_ok:
            return None
        d_lift = self.lift_dir * APPROACH_STEP if lift_ok else 0
        if not elbow_ok:
            d_elbow = 0
        elif lift_ok:
            d_elbow = self.elbow_dir * int(round(APPROACH_STEP * self.elbow_ratio))
        else:
            d_elbow = self.elbow_dir * APPROACH_STEP   # shoulder maxed: elbow reaches alone
        return d_lift, d_elbow

    def clamp_lift(self, cur, new):
        return self._clamp_toward(cur, new, self.lift_limit, self.lift_dir)

    def clamp_elbow(self, cur, new):
        return self._clamp_toward(cur, new, self.elbow_limit, self.elbow_dir)

    @staticmethod
    def _clamp_toward(cur, new, limit, d):
        # Stop at the limit in the approach direction, but never pull a joint
        # that is already past it backwards.
        if d > 0:
            return _clamp(new, hi=max(limit, cur))
        return _clamp(new, lo=min(limit, cur))

    def describe(self):
        return (f"approach from {self.source}: lift {'+' if self.lift_dir > 0 else '-'} "
                f"to {self.lift_limit}, elbow {'+' if self.elbow_dir > 0 else '-'}"
                f"{self.elbow_ratio:.2f}x to {self.elbow_limit}")

    def blocked_msg(self, lift, elbow):
        return (f"OUT-OF-REACH: lift={lift} (limit {self.lift_limit}) "
                f"elbow={elbow} (limit {self.elbow_limit})")


def open_gripper_firmly(arm, target=GRIPPER_TRACK_POS, label="ball"):
    """Open the jaws to ``target`` and make sure they actually opened.

    Suction-cup jaw tips that closed on nothing stick together, and a normal
    open stalls against the seal. If the jaws are still shut after the open,
    squeeze briefly and snap open at full speed, up to GRIPPER_UNSTICK_TRIES
    times. Returns the final jaw position (None if it can't be read)."""
    arm.setPosition(SERVO_GRIPPER, target, duration=300, wait=True)
    pos = stuck_at = None
    for attempt in range(GRIPPER_UNSTICK_TRIES + 1):
        time.sleep(0.15)
        try:
            pos = int(arm.getPosition(SERVO_GRIPPER))
        except Exception as e:
            print(f"[{label}] gripper read failed while opening: {e}")
            return None
        if pos <= target + GRIPPER_OPEN_SLACK:
            if attempt:
                print(f"[{label}] jaws unstuck after {attempt} snap(s): at {pos}")
            return pos
        if attempt == GRIPPER_UNSTICK_TRIES:
            break
        print(f"[{label}] jaws stuck at {pos} (wanted {target}) — snapping open")
        if stuck_at is None:
            stuck_at = pos       # squeeze from here every time, so it doesn't creep shut
        arm.setPosition(SERVO_GRIPPER, min(1000, stuck_at + GRIPPER_SQUEEZE), duration=100, wait=True)
        arm.setPosition(SERVO_GRIPPER, target, duration=GRIPPER_SNAP_MS, wait=True)
    print(f"[{label}] WARNING: jaws still stuck at {pos} after {GRIPPER_UNSTICK_TRIES} snaps "
          f"— pull the suction cups apart by hand")
    return pos


def radius_tolerance(target_radius_px):
    """+/- px around the target radius that counts as 'at grab distance':
    15% of it (33 px at the Tria's r=220, ~10 px at r=68), at least 5 px."""
    return max(5, int(round(RADIUS_TOLERANCE_FRAC * target_radius_px)))


def _clamp(v, lo=0, hi=1000):
    return max(lo, min(hi, int(v)))


def _step_toward(error_px, gain, max_step, min_step=MIN_TRIM_STEP):
    """Convert a pixel error to a clamped servo delta (sign preserved).

    Floors the magnitude at ``min_step`` so trims aren't silently swallowed by
    bus-servo static friction. Caller still gates on a separate deadband check
    before invoking this, so the floor only applies when we've already decided
    a move is warranted.
    """
    raw = error_px * gain
    if raw > max_step:
        return max_step
    if raw < -max_step:
        return -max_step
    if raw > 0 and raw < min_step:
        return min_step
    if raw < 0 and raw > -min_step:
        return -min_step
    return raw


class BallFollowMode(Mode):
    name = "ball"

    def __init__(self):
        self.lower = None
        self.upper = None
        self.frame_count = 0
        self.state = "IDLE"
        self.last_log = ""
        self.hold_target = None  # gripper position we're holding at; None = not holding
        self.no_ball_frames = 0  # consecutive frames with no ball detected
        self.pos_buffer = deque(maxlen=POS_BUFFER_LEN)  # (frame_idx, bx, by, br)
        self.pred_x = 0.0
        self.pred_y = 0.0
        self.pred_r = 0.0
        self.pred_frames_remaining = 0
        self.last_vel = (0.0, 0.0)
        self.last_ball = (0, 0, 0)   # most recent (bx, by, br), real or predicted
        self.last_errs = (0, 0, 0)   # pan_err, tilt_err, radius_err
        self.last_deltas = (0, 0, 0, 0)  # d_pan, d_tilt, d_lift, d_elbow
        self.last_is_prediction = False
        self.last_telemetry_at = 0.0
        # Scan-pose override system (scan_poses.json written by the
        # teach_scan_pose cloud command) — same as PickPlaceMode. Falls back
        # to the hardcoded SCAN_POSES when nothing has been taught.
        from core import scan_poses_store
        loaded_poses, loaded_labels = scan_poses_store.load()
        if loaded_poses:
            self.scan_poses = loaded_poses
            self.scan_pose_labels = loaded_labels
            print(f"[ball] using {len(loaded_poses)} scan poses from disk: {loaded_labels}")
        else:
            self.scan_poses = SCAN_POSES
            self.scan_pose_labels = SCAN_POSE_LABELS
        self.pan_min, self.pan_max = pan_limits(self.scan_poses)
        if (self.pan_min, self.pan_max) != (PAN_MIN, PAN_MAX):
            print(f"[ball] pan clamp widened to [{self.pan_min}, {self.pan_max}] to cover the scan poses")
        self.scan_idx = 0
        self.last_scan_move_at = 0.0
        # Camera-gripper calibration (cam_gripper_offset.json, written by the
        # calibrate_offset browser calibrator): where the ball should sit in
        # the image and how big it should look when tracking stops, plus the
        # optional taught reach from there to the grab pose. Falls back to the
        # module constants (no reach) when nothing has been calibrated.
        from core import cam_gripper_offset_store
        ofs = cam_gripper_offset_store.load() or {}
        self.offset_x = int(ofs.get('cam_gripper_offset_x', CAM_GRIPPER_OFFSET_X))
        self.offset_y = int(ofs.get('cam_gripper_offset_y', CAM_GRIPPER_OFFSET_Y))
        self.target_radius_px = int(ofs.get('target_radius_px', TARGET_RADIUS_PX))
        self.radius_tol = radius_tolerance(self.target_radius_px)
        self.grab_reach = cam_gripper_offset_store.load_reach()
        self.view_pose = ofs.get('view_pose')
        self.approach = ApproachPlan(self.scan_poses, self.view_pose, self.grab_reach)
        print(f"[ball] {self.approach.describe()}")
        if ofs:
            print(f"[ball] cam-gripper calibration: offset=({self.offset_x:+d},{self.offset_y:+d}) "
                  f"radius={self.target_radius_px}px reach={self.grab_reach or 'none (grab in place)'}")
        # perf instrumentation (printed from process_frame every PERF_EVERY frames)
        self._perf_n = 0
        self._perf_detect = 0.0
        self._perf_arm = 0.0
        self._perf_every = 30

    def setup(self, arm):
        if not os.path.exists(CONFIG_PATH):
            raise RuntimeError(f"ball_color.json not found at {CONFIG_PATH}; run calibration/ball_calibrate.py first")
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
        self.lower = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]], dtype=np.uint8)
        self.upper = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]], dtype=np.uint8)
        print(f"[ball] HSV range loaded: lower={self.lower.tolist()} upper={self.upper.tolist()}")
        print(f"[ball] moving to scan pose [{self.scan_pose_labels[0]}]...")
        arm.setPosition(self.scan_poses[0], duration=1500, wait=True)
        self.scan_idx = 0
        self.last_scan_move_at = time.time()
        self.state = "IDLE"

    def process_frame(self, frame, arm):
        from main import send_telemetry  # lazy to avoid circular import

        self.frame_count += 1
        h, w = frame.shape[:2]
        # Aim at the pixel where the ball appears at the calibrated view pose,
        # not the geometric image center (see cam_gripper_offset_store).
        cx_target = w // 2 + self.offset_x
        cy_target = h // 2 + self.offset_y

        # Single batched USB read for all 6 servos, reused below.
        # If even the per-servo fallback inside _read_all_positions fails,
        # skip this frame — commanding setPosition with bogus "current" values
        # (e.g. all 500s) would make the arm jump.
        try:
            pos = _read_all_positions(arm)
        except Exception as e:
            print(f"[ball] position read failed entirely — skipping frame: {e}")
            return cv2.flip(frame, 1)

        # If we're currently holding something, skip tracking entirely.
        # User can open the gripper (via IoTConnect) to release and resume.
        if self.hold_target is not None:
            gpos = pos[SERVO_GRIPPER]
            if gpos < self.hold_target - GRIPPER_RELEASE_DELTA:
                self._log(f"RELEASED: gripper {gpos} < hold_target {self.hold_target} - {GRIPPER_RELEASE_DELTA}")
                self.hold_target = None
            else:
                annotated = cv2.flip(frame, 1)
                cv2.putText(annotated, f"HOLDING @ {gpos} — tracking paused",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 255, 255), 2, cv2.LINE_AA)
                self._log(f"HOLDING: pos={gpos} target={self.hold_target}")
                self.state = "HOLDING"
                self._maybe_send_telemetry(arm, send_telemetry, positions=pos)
                return annotated

        t_detect_start = time.perf_counter()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        real_ball = self._largest_blob(mask)
        t_detect = time.perf_counter() - t_detect_start
        is_prediction = False
        ball = real_ball
        if real_ball is None:
            predicted = self._maybe_predict()
            if predicted is not None:
                ball = predicted
                is_prediction = True
        else:
            # Real observation — record it and cancel any in-progress prediction.
            self.pos_buffer.append((self.frame_count, real_ball[0], real_ball[1], real_ball[2]))
            self.pred_frames_remaining = 0
            # If we were scanning and just acquired the ball, halt the in-flight
            # scan move (which has duration=SCAN_MOVE_MS — could still be panning
            # for another 1-2s). Without this, the arm keeps sweeping past the
            # ball before the small per-frame TRACKING commands can take hold,
            # producing the "overshoot then chase back" pattern.
            if self.no_ball_frames > 0:
                try:
                    halt_targets = [
                        [SERVO_SHOULDER_PAN, pos[SERVO_SHOULDER_PAN]],
                        [SERVO_WRIST_FLEX, pos[SERVO_WRIST_FLEX]],
                        [SERVO_SHOULDER_LIFT, pos[SERVO_SHOULDER_LIFT]],
                        [SERVO_ELBOW_FLEX, pos[SERVO_ELBOW_FLEX]],
                    ]
                    arm.setPosition(halt_targets, duration=80, wait=False)
                    print(f"[ball] HALT scan: pan={pos[SERVO_SHOULDER_PAN]} "
                          f"flex={pos[SERVO_WRIST_FLEX]} lift={pos[SERVO_SHOULDER_LIFT]} "
                          f"elbow={pos[SERVO_ELBOW_FLEX]}")
                except Exception as e:
                    print(f"[ball] halt failed: {e}")
            self.no_ball_frames = 0

        # Flip first so all text we draw below reads the right way around.
        # Ball x-coords must be mirrored too: bx_disp = w - bx.
        annotated = cv2.flip(frame, 1)
        # Mirror the target x for display since the frame is flipped horizontally.
        cv2.drawMarker(annotated, (w - cx_target, cy_target), (255, 255, 255),
                       markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)

        if self.state == "GRABBING":
            cv2.putText(annotated, "GRABBING...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            return annotated

        if ball is None:
            # truly lost — nothing real, nothing to predict from
            self.no_ball_frames += 1
            label = self.scan_pose_labels[self.scan_idx]
            self._log(f"SCAN[{label}]: no ball ({self.no_ball_frames})")
            cv2.putText(annotated, f"scan {label}  no ball ({self.no_ball_frames})", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            self.state = "SCANNING"
            self.last_is_prediction = False
            # After a brief grace period, cycle scan poses to search the table.
            # wait=False so the camera loop keeps reading frames during the move
            # (otherwise video freezes for SCAN_MOVE_MS each transition). We pace
            # transitions ourselves: don't advance until the prior move's expected
            # completion time has passed plus SCAN_DWELL_S of viewing time.
            if self.no_ball_frames >= NO_BALL_GRACE_FRAMES:
                now = time.time()
                next_advance_at = self.last_scan_move_at + (SCAN_MOVE_MS / 1000.0) + SCAN_DWELL_S
                if now >= next_advance_at:
                    self.scan_idx = (self.scan_idx + 1) % len(self.scan_poses)
                    next_label = self.scan_pose_labels[self.scan_idx]
                    print(f"[ball] scanning -> {next_label}")
                    try:
                        arm.setPosition(self.scan_poses[self.scan_idx], duration=SCAN_MOVE_MS, wait=False)
                    except Exception as e:
                        print(f"[ball] scan move failed: {e}")
                    self.last_scan_move_at = time.time()
            self._maybe_send_telemetry(arm, send_telemetry, positions=pos)
            return annotated

        bx, by, br = ball
        bx_disp = w - bx
        color = (0, 165, 255) if is_prediction else (0, 255, 0)  # orange vs green
        cv2.circle(annotated, (bx_disp, by), int(br), color, 2)
        cv2.circle(annotated, (bx_disp, by), 4, color, -1)
        if is_prediction:
            cv2.putText(annotated, f"PREDICTING ({self.pred_frames_remaining} left)",
                        (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 165, 255), 2, cv2.LINE_AA)
        pan_err = bx - cx_target
        tilt_err = by - cy_target
        radius_err = self.target_radius_px - br
        centered_ok = abs(pan_err) <= CENTER_DEADBAND_PX and abs(tilt_err) <= GRAB_TILT_DEADBAND_PX
        # Looser gate that just requires the ball to be roughly under the gripper:
        # used to allow descent while pan/tilt are still trimming, so we don't
        # sit forever waiting for perfect centering before reaching for the ball.
        approach_centered_ok = abs(pan_err) <= APPROACH_DEADBAND_PX and abs(tilt_err) <= APPROACH_DEADBAND_PX
        radius_ok = abs(radius_err) <= self.radius_tol
        cv2.putText(annotated,
                    f"r={int(br)} dx={pan_err:+d} dy={tilt_err:+d}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(annotated,
                    f"center={'OK' if centered_ok else 'NO'}  radius={'OK' if radius_ok else 'NO'} (target={self.target_radius_px}+/-{self.radius_tol})",
                    (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if centered_ok and radius_ok else (0, 200, 255),
                    2, cv2.LINE_AA)

        # Are we close + centered enough to grab? Only on real observations —
        # never close the gripper on a predicted position.
        if not is_prediction and centered_ok and radius_ok:
            self.state = "GRABBING"
            self._log(f"GRAB: centered and at distance — r={int(br)} (taught {self.target_radius_px}) "
                      f"dx={pan_err:+d} dy={tilt_err:+d} — closing gripper")
            self.state = "HOLDING" if self._do_grab(arm) else "TRACKING"
            try:
                send_telemetry(arm, extras={"ballTrack": self.telemetry()}, positions=pos)
                self.last_telemetry_at = time.time()
            except Exception as e:
                print(f"[ball] telemetry after grab failed: {e}")
            return annotated

        # Throttle servo commands so we don't spam at full frame rate.
        if self.frame_count % UPDATE_EVERY != 0:
            return annotated

        self.state = "PREDICTING" if is_prediction else "TRACKING"
        d_pan = 0
        d_tilt = 0
        d_lift = 0
        d_elbow = 0

        if abs(pan_err) > CENTER_DEADBAND_PX:
            d_pan = int(round(_step_toward(pan_err, PAN_GAIN, MAX_STEP,
                                            min_step=MIN_TRIM_STEP_PAN))) * PAN_DIR
        if abs(tilt_err) > GRAB_TILT_DEADBAND_PX:
            d_tilt = int(round(_step_toward(tilt_err, TILT_GAIN, MAX_STEP))) * TILT_DIR
            # Elbow helps with larger tilts so wrist_flex doesn't max out alone.
            d_elbow += int(round(d_tilt * TILT_ELBOW_RATIO)) * TILT_ELBOW_DIR
        # Allow descent as soon as the ball is roughly under the gripper
        # (looser approach_centered_ok). Pan/tilt corrections continue every
        # frame and trim the ball back toward the gripper aim point even while
        # we descend, so we don't wait forever for perfect centering.
        if approach_centered_ok and br < self.target_radius_px - self.radius_tol:
            # Hard ceiling: if we're already at the safety limit, stop approaching.
            # Prevents table slam when radius_ok never triggers.
            step = self.approach.step(pos[SERVO_SHOULDER_LIFT], pos[SERVO_ELBOW_FLEX])
            if step is None:
                self._log(f"{self.approach.blocked_msg(pos[SERVO_SHOULDER_LIFT], pos[SERVO_ELBOW_FLEX])}; "
                          f"r={int(br)} target={self.target_radius_px}")
            else:
                # Reach toward the ball: shoulder and elbow together, in the
                # direction that extends THIS arm (see ApproachPlan).
                d_lift, d_reach_elbow = step
                d_elbow += d_reach_elbow
                # Trim the wrist with every reach step, even inside the deadband. Each lift step shifts the ball
                # ~30 px in the image (~5 px per lift unit); recentering in the same command keeps the ball
                # in view instead of alternating reach and recenter frames.
                # Pan is not trimmed here: reaching barely moves the ball sideways, and a floored pan step
                # (~126 px) would overshoot the deadband and oscillate.
                if d_tilt == 0 and tilt_err != 0:
                    d_tilt = int(round(_step_toward(tilt_err, TILT_GAIN, MAX_STEP))) * TILT_DIR

        if d_pan == 0 and d_tilt == 0 and d_lift == 0 and d_elbow == 0:
            return annotated

        targets = []
        if d_pan:
            new_pan = _clamp(pos[SERVO_SHOULDER_PAN] + d_pan, lo=self.pan_min, hi=self.pan_max)
            targets.append([SERVO_SHOULDER_PAN, new_pan])
        if d_tilt:
            new_tilt = _clamp(pos[SERVO_WRIST_FLEX] + d_tilt)
            targets.append([SERVO_WRIST_FLEX, new_tilt])
        if d_lift:
            new_lift = self.approach.clamp_lift(pos[SERVO_SHOULDER_LIFT],
                                                pos[SERVO_SHOULDER_LIFT] + d_lift)
            targets.append([SERVO_SHOULDER_LIFT, new_lift])
        if d_elbow:
            new_elbow = self.approach.clamp_elbow(pos[SERVO_ELBOW_FLEX],
                                                  pos[SERVO_ELBOW_FLEX] + d_elbow)
            targets.append([SERVO_ELBOW_FLEX, new_elbow])

        t_arm_start = time.perf_counter()
        if targets:
            self._log(f"{self.state}: dpan={d_pan} dtilt={d_tilt} dlift={d_lift} delbow={d_elbow}")
            try:
                # wait=False so the camera loop doesn't freeze for ~MOVE_DURATION_MS
                # while the servo travels. Bus servos accept a new target while a
                # prior move is in flight — they just retarget smoothly. Each new
                # frame issues a fresh small correction based on the latest pixel
                # error, which is what we want.
                arm.setPosition(targets, duration=MOVE_DURATION_MS, wait=False)
            except Exception as e:
                print(f"[ball] setPosition failed: {e}")
        t_arm = time.perf_counter() - t_arm_start
        self._perf_n += 1
        self._perf_detect += t_detect
        self._perf_arm += t_arm
        if self._perf_n >= self._perf_every:
            print(f"[ball-perf] n={self._perf_n}  detect={self._perf_detect/self._perf_n*1000:.1f}ms  "
                  f"arm={self._perf_arm/self._perf_n*1000:.1f}ms")
            self._perf_n = 0
            self._perf_detect = 0.0
            self._perf_arm = 0.0

        # snapshot state for telemetry
        self.last_ball = (int(bx), int(by), int(br))
        self.last_errs = (int(pan_err), int(tilt_err), int(radius_err))
        self.last_deltas = (int(d_pan), int(d_tilt), int(d_lift), int(d_elbow))
        self.last_is_prediction = is_prediction
        self._maybe_send_telemetry(arm, send_telemetry, positions=pos)

        return annotated

    @staticmethod
    def _largest_blob(mask):
        """Pick the largest blob that's also round-ish.

        A ball's contour fills its minimum enclosing circle by ~85% or more.
        Irregular ball-color objects (a logo, a corner of fabric, a shadow
        edge) typically fill <50% of their enclosing circle even if their
        raw area is large. The fill_ratio filter rejects those false-positive
        winners that the area-only check used to accept.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        best_area = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA or area <= best_area:
                continue
            (x, y), r = cv2.minEnclosingCircle(c)
            if r <= 0:
                continue
            fill_ratio = area / (np.pi * r * r)
            if fill_ratio < MIN_FILL_RATIO:
                continue
            best = (int(x), int(y), float(r))
            best_area = area
        return best

    def _do_grab(self, arm):
        """Open fully, (if taught) reach from the view pose to the grab pose,
        close on the ball, wait for the jaws to stop, lift and return home.
        Home pose deliberately excludes the gripper so the ball isn't dropped.

        The reach (cam_gripper_offset.json ``grab_reach``) is for mounts where
        the camera can't see the ball once it is between the jaws: tracking
        stops at the taught view pose and this blind, taught joint move puts
        the jaws around the ball.

        Returns True if we are holding the ball after the lift. If the jaws
        closed on nothing, or are (nearly) shut after the lift (ball slipped
        out), the gripper is re-opened, hold_target stays None and False is
        returned so the caller goes back to tracking instead of carrying an
        empty gripper to the box.
        """
        open_gripper_firmly(arm, 0)
        view_pose = self._reach_to_grab_pose(arm)
        arm.setPosition(SERVO_GRIPPER, GRIPPER_CLOSE_TARGET, duration=GRIPPER_CLOSE_MS, wait=True)

        # wait=True only sleeps for the commanded duration; closing on the
        # ball the servo runs slower than that, so poll until the jaws have
        # actually stopped before deciding where they stalled and lifting.
        actual = self._wait_gripper_settled(arm)
        if actual >= GRIPPER_CLOSE_TARGET - GRIPPER_STALL_SLACK:
            print(f"[ball] gripper closed to {actual} — nothing in the jaws; back to tracking")
            open_gripper_firmly(arm, GRIPPER_TRACK_POS)
            if view_pose:
                # Back to where the camera could see the ball, so tracking
                # can find it again instead of staring past it.
                arm.setPosition(view_pose, duration=GRAB_REACH_MS, wait=True)
            return False

        # Stalled on the ball. Leave the servo commanded at full close so it
        # keeps pushing at maximum torque for the whole carry. Sending a new
        # "relaxed" target here makes the servo re-interpolate from where it
        # currently sits, which momentarily unloads the jaws and lets the
        # ball shift. hold_target is where the jaws actually stopped — the
        # release check compares the live position to it.
        print(f"[ball] gripper stalled at {actual} (target {GRIPPER_CLOSE_TARGET}); holding at full close")
        self.hold_target = actual

        arm.setPosition([
            [SERVO_SHOULDER_LIFT, 300],
            [SERVO_ELBOW_FLEX, 350],
        ], duration=1500, wait=True)

        # If the ball squirted out on the way up the jaws will have snapped
        # (nearly) shut. Catch that here rather than transporting an empty
        # gripper. The threshold is absolute: the jaws keep creeping closed
        # on a held ball for a couple of seconds, so "closed further than the
        # stall" is normal and does NOT mean the ball is gone.
        try:
            after = int(arm.getPosition(SERVO_GRIPPER))
        except Exception as e:
            print(f"[ball] gripper read after lift failed: {e}")
            after = actual
        if after >= GRIPPER_EMPTY_POS:
            print(f"[ball] ball slipped during lift: gripper {actual} -> {after}; back to tracking")
            self.hold_target = None
            open_gripper_firmly(arm, GRIPPER_TRACK_POS)
            return False
        print(f"[ball] lifted; gripper {actual} -> {after}")

        arm.setPosition(HOME_POSE_KEEP_GRIP, duration=1800, wait=True)
        return True

    def _reach_to_grab_pose(self, arm):
        """Apply the taught grab reach relative to the current pose. Returns
        the pre-reach pose ([[servo, pos], ...]) so a missed grab can back
        out to it, or None when no reach is taught / the pose can't be read."""
        if not self.grab_reach:
            return None
        servos = set(self.grab_reach) | set(arm_kinematics.CHAIN)
        try:
            here = {sid: int(arm.getPosition(sid)) for sid in servos}
        except Exception as e:
            print(f"[ball] reach skipped — position read failed: {e}")
            return None
        # Default: the taught joint deltas. Pan / wrist roll always use them.
        target = {sid: _clamp(here[sid] + d) for sid, d in self.grab_reach.items()}
        how = "joint deltas"
        # Shoulder / elbow / wrist: the same move relative to the GRIPPER as
        # taught, solved from the arm's current shape (see arm_kinematics).
        if self.view_pose:
            view = {int(k): v for k, v in self.view_pose.items()}
            solved = arm_kinematics.gripper_frame_reach(
                view, self.grab_reach, here,
                arm_kinematics.joint_signs(self.approach.mirrored))
            if solved:
                target.update(solved)
                how = "gripper-frame move"
            else:
                how = "joint deltas (gripper-frame move unsolvable from here)"
        print(f"[ball] reaching to grab pose ({how}): "
              f"{ {s: here[s] for s in target} } -> {target}")
        arm.setPosition([[s, p] for s, p in target.items()], duration=GRAB_REACH_MS, wait=True)
        return [[sid, here[sid]] for sid in target]

    def _wait_gripper_settled(self, arm):
        """Poll the gripper until GRIPPER_SETTLE_READS consecutive reads agree
        (jaws stopped — stalled on the ball or fully closed), then dwell so
        the servo is at full torque. Returns the final position. A read
        failure ends the wait early with the last value we got."""
        trace = [int(arm.getPosition(SERVO_GRIPPER))]
        deadline = time.time() + GRIPPER_SETTLE_TIMEOUT_S
        stable = 0
        while time.time() < deadline:
            time.sleep(0.1)
            try:
                cur = int(arm.getPosition(SERVO_GRIPPER))
            except Exception as e:
                print(f"[ball] gripper read failed while settling: {e}")
                break
            stable = stable + 1 if abs(cur - trace[-1]) <= GRIPPER_SETTLE_DELTA else 0
            trace.append(cur)
            if stable >= GRIPPER_SETTLE_READS - 1:
                break
        else:
            print(f"[ball] gripper still moving after {GRIPPER_SETTLE_TIMEOUT_S}s")
        print(f"[ball] gripper settle trace: {' '.join(map(str, trace))}")
        time.sleep(GRIPPER_SETTLE_DWELL_S)
        return trace[-1]

    def _maybe_predict(self):
        """Return a (bx, by, br) tuple extrapolated from recent motion, or None.

        On first call after a ball loss, estimates velocity from self.pos_buffer
        and seeds pred_x/pred_y. Subsequent calls step the prediction forward by
        the estimated per-frame velocity. Runs for at most MAX_PREDICT_FRAMES.
        """
        if not ENABLE_PREDICTION:
            return None
        if self.pred_frames_remaining > 0:
            self.pred_x += self.last_vel[0]
            self.pred_y += self.last_vel[1]
            self.pred_frames_remaining -= 1
            return (int(self.pred_x), int(self.pred_y), self.pred_r)

        if len(self.pos_buffer) < 2:
            return None
        fi_old, bx_old, by_old, _ = self.pos_buffer[0]
        fi_new, bx_new, by_new, br_new = self.pos_buffer[-1]
        dt = fi_new - fi_old
        if dt <= 0:
            return None
        vx = (bx_new - bx_old) / dt
        vy = (by_new - by_old) / dt
        if vx * vx + vy * vy < MIN_VELOCITY_PX_PER_FRAME ** 2:
            return None
        # account for frames that have already elapsed since the last real obs
        frames_since = max(1, self.frame_count - fi_new)
        self.last_vel = (vx, vy)
        self.pred_x = bx_new + vx * frames_since
        self.pred_y = by_new + vy * frames_since
        self.pred_r = br_new
        self.pred_frames_remaining = MAX_PREDICT_FRAMES - 1
        print(f"[ball] predicting: v=({vx:+.1f},{vy:+.1f}) px/frame, {MAX_PREDICT_FRAMES} frames")
        return (int(self.pred_x), int(self.pred_y), self.pred_r)

    def _maybe_send_telemetry(self, arm, send_telemetry, positions=None):
        now = time.time()
        if now - self.last_telemetry_at < TELEMETRY_INTERVAL_S:
            return
        try:
            send_telemetry(arm, extras={"ballTrack": self.telemetry()}, positions=positions)
            self.last_telemetry_at = now
        except Exception as e:
            print(f"[ball] telemetry failed: {e}")

    def get_state(self):
        return self.state

    def telemetry(self):
        """Build a flat dict of mode state for IoTConnect telemetry."""
        bx, by, br = self.last_ball
        pan_err, tilt_err, radius_err = self.last_errs
        d_pan, d_tilt, d_lift, d_elbow = self.last_deltas
        vx, vy = self.last_vel
        return {
            "state": self.state,
            "is_prediction": 1 if self.last_is_prediction else 0,
            "ball_x": bx,
            "ball_y": by,
            "ball_r": br,
            "pan_err": pan_err,
            "tilt_err": tilt_err,
            "radius_err": radius_err,
            "velocity_x": round(float(vx), 2),
            "velocity_y": round(float(vy), 2),
            "pred_frames_left": int(self.pred_frames_remaining),
            "no_ball_frames": int(self.no_ball_frames),
            "hold_target": int(self.hold_target) if self.hold_target is not None else 0,
            "d_pan": d_pan,
            "d_tilt": d_tilt,
            "d_lift": d_lift,
            "d_elbow": d_elbow,
        }

    def _log(self, msg):
        if msg != self.last_log:
            print(f"[ball] {msg}")
            self.last_log = msg
