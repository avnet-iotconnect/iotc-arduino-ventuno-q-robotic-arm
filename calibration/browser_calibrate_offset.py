#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Browser-based camera-gripper calibrator: view pose + grab reach.

Rendered as MJPEG + a small HTML page, so it works from any browser on the
LAN. Workflow:

  1. SUPPORT THE ARM — all torque drops on launch.
  2. Open http://<board-ip>:<port>/ in a browser.
  3. STEP 1 — view pose. Pose the arm where it should stop tracking: gripper
     just above/behind the ball, ball FULLY in the camera view. Hold steady,
     click "1. Snapshot view pose". Averages the last ~30 frames and saves
     the ball's pixel offset from image center, its apparent radius, and the
     arm's joint pose to cam_gripper_offset.json. When the YOLO + MiDaS
     models are installed it also teaches the YOLO grab depth here (the
     same thing `calibrate grab_depth` does): grab_depth.json + the grab
     gate's depth floor.
  4. STEP 2 — grab pose (optional). Without moving the ball, move the arm
     so the open jaws are around the ball (the camera does not need to see
     it), click "2. Snapshot grab pose". Saves the joint move from the view
     pose to here; at grab time the arm makes that move blind, then closes.
     Skip step 2 if the ball stays visible between the jaws — the gripper
     then closes right at the view pose.
  5. "Hold pose" re-engages torque. "Release torque" drops it again.

The ball / pickplace / yolo modes load the file when they start.
"""

import argparse
import json
import os
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Lives in a subfolder: put the repo root on the path so core/, modes/,
# detectors/ and main.py import.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2
import numpy as np
import xarm

from core import camera_settings as cam_settings
from core import cam_gripper_offset_store
from core import grab_threshold_store

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HSV_PATH = os.path.join(HERE, "ball_color.json")

# Match ball_follow.py detection params so what we measure matches what it sees.
MIN_CONTOUR_AREA = 200
MIN_FILL_RATIO = 0.65
SAMPLE_WINDOW = 30
ALL_SERVO_IDS = [1, 2, 3, 4, 5, 6]


def _median(values):
    s = sorted(values)
    n = len(s)
    if n == 0:
        raise ValueError("median of empty sequence")
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2


def _pstdev(values):
    n = len(values)
    if n == 0:
        return 0.0
    mean = sum(values) / n
    return (sum((v - mean) ** 2 for v in values) / n) ** 0.5


def largest_blob(mask):
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
        if area / (np.pi * r * r) < MIN_FILL_RATIO:
            continue
        best = (int(x), int(y), float(r))
        best_area = area
    return best


# ---------------------------- shared state ----------------------------

class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.frame_jpeg = b""
        self.frame_w = 0
        self.frame_h = 0
        self.bx = self.by = self.br = None      # latest detection
        self.recent = deque(maxlen=SAMPLE_WINDOW)  # (bx, by, br)
        self.last_snapshot = None               # dict shown on the page
        self.quit = False
        self.arm = None                         # set in main(); used to read joint poses
        self.depth_recent = deque(maxlen=SAMPLE_WINDOW)  # (D, yolo_r) — YOLO grab-depth teach
        self.depth_models = None                # (model_path, depth_path) when YOLO teach is on


# ---------------------------- camera loop -----------------------------

def _load_depth_teach(use_npu=True):
    """YOLO ball detector + MiDaS, same models/conf as the yolo modes, so
    the view-pose snapshot can also teach the YOLO grab depth. Returns
    (ball_det, depth_det, model_path, depth_path) or None if unavailable."""
    try:
        from modes import yolo_model_choice, MIDAS_MODEL
        model_path, conf = yolo_model_choice()
        if not (os.path.exists(model_path) and os.path.exists(MIDAS_MODEL)):
            print("[calib] YOLO/MiDaS models not installed — grab depth not taught here")
            return None
        from detectors import make_ball_detector, DepthDetector
        print("[calib] loading YOLO + MiDaS for grab-depth teach...")
        ball_det = make_ball_detector(model_name=model_path, conf_thres=conf, use_npu=use_npu)
        depth_det = DepthDetector(MIDAS_MODEL, use_npu=use_npu)
        return ball_det, depth_det, model_path, MIDAS_MODEL
    except Exception as e:
        print(f"[calib] grab-depth teach unavailable ({e}) — offset/reach only")
        return None


def camera_loop(state, camera_index, width, height, lower, upper,
                camera_settings_path=None, jpeg_quality=80, depth_teach=None):
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    settings_path = camera_settings_path or cam_settings.DEFAULT_PATH
    cam_settings.apply(cap, cam_settings.load(settings_path), path=settings_path)
    if not cap.isOpened():
        print(f"[calib] ERROR: could not open camera {camera_index}", file=sys.stderr)
        state.quit = True
        return
    print(f"[calib] camera {camera_index} opened ({width}x{height})")
    enc_params = [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality]

    try:
        while not state.quit:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.01)
                continue
            h, w = frame.shape[:2]

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower, upper)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

            ball = largest_blob(mask)

            # YOLO + depth at the ball (for the grab-depth teach), measured
            # exactly as the yolo modes measure it.
            ydet = D = None
            if depth_teach is not None:
                ball_det, depth_det = depth_teach[0], depth_teach[1]
                dets = ball_det.detect(frame)
                ydet = max(dets, key=lambda d: d.r) if dets else None
                if ydet is not None:
                    dm = depth_det.infer(frame)
                    D = depth_det.at(dm, ydet.cx, ydet.cy, patch_r=max(5, int(ydet.r / 3)))

            # Annotate. Mirror horizontally so what the user sees matches
            # how ball_follow displays the wrist camera.
            annotated = cv2.flip(frame, 1)
            cx, cy = w // 2, h // 2
            cv2.drawMarker(annotated, (cx, cy), (255, 255, 255),
                           markerType=cv2.MARKER_CROSS, markerSize=20, thickness=1)

            with state.lock:
                state.frame_w, state.frame_h = w, h
                if ball is not None:
                    bx, by, br = ball
                    state.bx, state.by, state.br = bx, by, br
                    state.recent.append((bx, by, br))
                    dx = bx - cx
                    dy = by - cy
                    bx_disp = w - bx
                    cv2.circle(annotated, (bx_disp, by), int(br), (0, 255, 0), 2)
                    cv2.circle(annotated, (bx_disp, by), 4, (0, 255, 0), -1)
                    cv2.putText(annotated, f"bx={bx} by={by} r={int(br)}",
                                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.putText(annotated,
                                f"OFFSET_X={dx:+d}  OFFSET_Y={dy:+d}",
                                (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 255), 2, cv2.LINE_AA)
                    cv2.putText(annotated,
                                f"samples={len(state.recent)}/{SAMPLE_WINDOW}",
                                (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (200, 200, 200), 1, cv2.LINE_AA)
                else:
                    state.bx = state.by = state.br = None
                    cv2.putText(annotated, "no ball detected", (10, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 0, 255), 2, cv2.LINE_AA)
                if depth_teach is not None:
                    if ydet is not None and D is not None:
                        state.depth_recent.append((float(D), float(ydet.r)))
                        ytxt, ycol = f"YOLO r={ydet.r:.0f}  depth D={D:.0f}", (255, 200, 0)
                    else:
                        ytxt, ycol = "YOLO: no ball (grab depth not measured)", (0, 0, 255)
                    cv2.putText(annotated, ytxt, (10, h - 15), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, ycol, 2, cv2.LINE_AA)

                ok2, buf = cv2.imencode(".jpg", annotated, enc_params)
                if ok2:
                    state.frame_jpeg = buf.tobytes()
    finally:
        cap.release()
        print("[calib] camera released")


# ---------------------------- HTTP plumbing ---------------------------

INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><title>Cam-Gripper Offset Calibrator</title>
<style>
  body { font: 14px sans-serif; margin: 1em; background: #111; color: #eee; }
  #wrap { display: flex; gap: 1em; flex-wrap: wrap; }
  #stream { display: block; max-width: 100%; }
  #panel { min-width: 320px; }
  button { margin: 0.2em 0.4em 0.2em 0; padding: 0.5em 0.9em; font-size: 14px; cursor: pointer; }
  pre { background: #222; padding: 0.6em; border-radius: 4px; max-height: 360px; overflow: auto; white-space: pre-wrap; }
  .hi { color: #6cf; }
</style></head>
<body>
<h2>Camera-gripper calibrator: view pose + grab reach</h2>
<div id="wrap">
  <div>
    <img id="stream" src="/stream" alt="(camera stream)">
    <p><b>Step 1:</b> pose the arm where it should stop tracking: gripper just above/behind the ball, with the ball <b>fully in view</b>. Keep holding the arm (torque off) and click <b>1. Snapshot view pose</b>. With the YOLO models installed this also teaches the YOLO grab depth (blue text at the bottom of the video).</p>
    <p><b>Step 2:</b> without moving the ball, move the arm so the open jaws are around it (the camera doesn't need to see it), then click <b>2. Snapshot grab pose</b>. Skip step 2 if the ball stays in view between the jaws.</p>
  </div>
  <div id="panel">
    <div>
      <button id="snap">1. Snapshot view pose (s)</button>
      <button id="grab">2. Snapshot grab pose (g)</button>
      <button id="hold">Hold pose (h)</button>
      <button id="release">Release torque (r)</button>
      <button id="quit">Quit</button>
    </div>
    <p id="status">live: (no ball)</p>
    <pre id="result">(no snapshot yet)</pre>
  </div>
</div>
<script>
const status = document.getElementById('status');
const result = document.getElementById('result');

async function action(path) {
  const r = await fetch(path, {method: 'POST'});
  const d = await r.json();
  refresh(d);
}
document.getElementById('snap').onclick    = () => action('/snapshot');
document.getElementById('grab').onclick    = () => action('/grab_snapshot');
document.getElementById('hold').onclick    = () => action('/hold');
document.getElementById('release').onclick = () => action('/release');
document.getElementById('quit').onclick    = () => { if (confirm('Quit?')) action('/quit'); };

document.addEventListener('keydown', (ev) => {
  if (ev.target.tagName === 'INPUT') return;
  if (ev.key === 's') action('/snapshot');
  if (ev.key === 'g') action('/grab_snapshot');
  if (ev.key === 'h') action('/hold');
  if (ev.key === 'r') action('/release');
});

function refresh(data) {
  if (!data) return;
  if (data.live) {
    if (data.live.bx === null) {
      status.textContent = `live: no ball  (samples ${data.live.samples}/${data.live.sample_window})`;
    } else {
      status.textContent = `live: bx=${data.live.bx} by=${data.live.by} r=${data.live.br}`
        + `  OFFSET_X=${data.live.off_x>=0?'+':''}${data.live.off_x}`
        + `  OFFSET_Y=${data.live.off_y>=0?'+':''}${data.live.off_y}`
        + `  (samples ${data.live.samples}/${data.live.sample_window})`;
    }
  }
  if (data.snapshot) result.textContent = data.snapshot;
  if (data.message) result.textContent = data.message + '\\n\\n' + result.textContent;
}

setInterval(async () => {
  try { refresh(await (await fetch('/state')).json()); } catch(e) {}
}, 500);
</script>
</body></html>
"""


def _live_view(state):
    with state.lock:
        bx, by, br = state.bx, state.by, state.br
        w, h = state.frame_w, state.frame_h
        n = len(state.recent)
    if not w or not h:
        return None
    cx, cy = w // 2, h // 2
    if bx is None:
        return {"bx": None, "by": None, "br": None,
                "off_x": 0, "off_y": 0,
                "samples": n, "sample_window": SAMPLE_WINDOW}
    return {"bx": int(bx), "by": int(by), "br": int(br),
            "off_x": int(bx - cx), "off_y": int(by - cy),
            "samples": n, "sample_window": SAMPLE_WINDOW}


def _state_payload(state, message=None, snapshot_text=None):
    payload = {"live": _live_view(state)}
    with state.lock:
        last = state.last_snapshot
    if snapshot_text is not None:
        payload["snapshot"] = snapshot_text
    elif last is not None:
        payload["snapshot"] = last
    if message is not None:
        payload["message"] = message
    return payload


def _read_pose(arm, servo_ids, reads=3):
    """Median of a few reads per servo (hand-posed arm, torque off).
    Returns {servo_id: pos} or None if the arm can't be read."""
    if arm is None:
        return None
    pose = {}
    for sid in servo_ids:
        vals = []
        for _ in range(reads):
            try:
                vals.append(int(arm.getPosition(sid)))
            except Exception as e:
                print(f"[calib] servo {sid} read failed: {e}")
            time.sleep(0.02)
        if not vals:
            return None
        pose[sid] = int(_median(vals))
    return pose


def _do_grab_snapshot(state):
    """Step 2: record the joint move from the saved view pose to here."""
    pose = _read_pose(state.arm, cam_gripper_offset_store.REACH_SERVOS)
    if pose is None:
        return "grab pose: could not read the arm's joint positions (is the arm connected?)"
    data = cam_gripper_offset_store.save_reach(pose)
    if data is None:
        return "grab pose: do STEP 1 (view pose snapshot) first — the reach is measured from it"
    names = {3: "wrist_flex", 4: "elbow", 5: "shoulder_lift", 6: "shoulder_pan"}
    lines = "\n".join(f"  {names[int(sid)]:<13} {int(d):+d}" for sid, d in data['grab_reach'].items())
    text = (
        "=== STEP 2 saved: grab reach (view pose -> grab pose) ===\n"
        f"{lines}\n\n"
        f"--- saved to {cam_gripper_offset_store.DEFAULT_PATH} ---\n"
        "At grab time the arm stops at the view pose, makes this move without\n"
        "the camera, then closes the gripper.\n"
        "# Send `set_mode ...` from the cloud to reload.\n"
    )
    with state.lock:
        state.last_snapshot = (state.last_snapshot or "") + "\n" + text
        full = state.last_snapshot
    print()
    print(text)
    return full


def _do_snapshot(state):
    with state.lock:
        samples = list(state.recent)
        w, h = state.frame_w, state.frame_h
    if len(samples) < 5:
        return f"not enough samples yet ({len(samples)}/5) — make sure ball is visible and steady"

    xs = [p[0] for p in samples]
    ys = [p[1] for p in samples]
    rs = [p[2] for p in samples]
    bx_med = int(_median(xs))
    by_med = int(_median(ys))
    r_med = float(_median(rs))
    cx, cy = w // 2, h // 2
    off_x = bx_med - cx
    off_y = by_med - cy
    bx_std = _pstdev(xs)
    by_std = _pstdev(ys)
    r_std = _pstdev(rs)

    # Persist to disk — ball + pickplace modes load this on mode start, so
    # one snapshot here actually changes the live behaviour the next time
    # those modes initialise (no SSH or file edit required).
    target_radius_int = int(round(r_med))
    view_pose = _read_pose(state.arm, cam_gripper_offset_store.REACH_SERVOS)
    cam_gripper_offset_store.save(off_x, off_y, target_radius_int, view_pose=view_pose)
    pose_note = (f"view pose (joints) = {view_pose}\n"
                 "Next: STEP 2 — move the open jaws around the ball, click\n"
                 "'2. Snapshot grab pose'. Skip it if the camera still sees the\n"
                 "ball between the jaws.\n"
                 if view_pose else
                 "(arm not readable — view pose not saved; STEP 2 unavailable)\n")
    depth_note = ""
    if state.depth_models is not None:
        with state.lock:
            dsamples = list(state.depth_recent)
        try:
            p, thr = grab_threshold_store.save_taught(
                [d for d, _ in dsamples], [r for _, r in dsamples],
                model=state.depth_models[0], depth_model=state.depth_models[1])
            depth_note = (f"YOLO grab depth taught: D_grab={p['D_grab']:.0f} "
                          f"(stdev {p['D_stdev']:.1f}), YOLO r={p['ball_r_at_grab']:.0f}px\n"
                          f"  -> grab gate now D >= {thr:g}  ({grab_threshold_store.GRAB_DEPTH_PATH})\n")
        except ValueError as e:
            depth_note = f"YOLO grab depth NOT taught: {e}\n"
    text = (
        f"=== STEP 1 saved: view pose, averaged over last {len(samples)} frames ===\n"
        f"bx = {bx_med}  (std {bx_std:.1f})\n"
        f"by = {by_med}  (std {by_std:.1f})\n"
        f"r  = {r_med:.1f}  (std {r_std:.1f})\n"
        f"image center = ({cx}, {cy})\n\n"
        f"--- saved to {cam_gripper_offset_store.DEFAULT_PATH} ---\n"
        f"cam_gripper_offset_x = {off_x}\n"
        f"cam_gripper_offset_y = {off_y}\n"
        f"target_radius_px     = {target_radius_int}\n"
        f"{depth_note}"
        f"{pose_note}"
        f"# (suggested RADIUS_TOLERANCE for ball_follow.py: +/-{max(15, int(round(r_std * 3)))})\n"
        f"# Send `set_mode ...` from the cloud to reload.\n"
    )
    with state.lock:
        state.last_snapshot = text
    print()
    print(text)
    return text


def make_handler(state, arm):

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            return

        def _json(self, payload, code=200):
            body = json.dumps(payload).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                body = INDEX_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == "/state":
                self._json(_state_payload(state))
                return
            if self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=FRAME")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.end_headers()
                last = None
                try:
                    while not state.quit:
                        with state.lock:
                            buf = state.frame_jpeg
                        if buf and buf is not last:
                            self.wfile.write(b"--FRAME\r\n")
                            self.wfile.write(b"Content-Type: image/jpeg\r\n")
                            self.wfile.write(
                                f"Content-Length: {len(buf)}\r\n\r\n".encode())
                            self.wfile.write(buf)
                            self.wfile.write(b"\r\n")
                            last = buf
                        time.sleep(0.05)
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return
            self.send_response(404)
            self.end_headers()

        def do_POST(self):
            if self.path == "/snapshot":
                text = _do_snapshot(state)
                self._json(_state_payload(state, snapshot_text=text))
                return
            if self.path == "/grab_snapshot":
                text = _do_grab_snapshot(state)
                self._json(_state_payload(state, snapshot_text=text))
                return
            if self.path == "/hold":
                if arm is None:
                    self._json(_state_payload(state, message="--no-arm: hold ignored"))
                    return
                try:
                    targets = [[sid, int(arm.getPosition(sid))] for sid in ALL_SERVO_IDS]
                    arm.setPosition(targets, duration=1500, wait=True)
                    msg = "torque ON at current pose"
                except Exception as e:
                    msg = f"hold failed: {e}"
                print(f"[calib] {msg}")
                self._json(_state_payload(state, message=msg))
                return
            if self.path == "/release":
                if arm is None:
                    self._json(_state_payload(state, message="--no-arm: release ignored"))
                    return
                try:
                    arm.servoOff()
                    msg = "ALL torque OFF — pose the gripper over the ball"
                except Exception as e:
                    msg = f"release failed: {e}"
                print(f"[calib] {msg}")
                self._json(_state_payload(state, message=msg))
                return
            if self.path == "/quit":
                state.quit = True
                self._json(_state_payload(state, message="exiting"))
                return
            self.send_response(404)
            self.end_headers()

    return Handler


def _local_ip_hint():
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "0.0.0.0"


def main():
    parser = argparse.ArgumentParser(description="Browser cam-gripper offset calibrator")
    parser.add_argument("--camera", type=int, default=None,
                        help="OpenCV camera index (default: auto-detect the USB camera)")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--port", type=int, default=8000, help="HTTP listen port")
    parser.add_argument("--no-prompt", action="store_true",
                        help="Skip the 'press Enter to release torque' safety prompt. "
                             "Use ONLY when launched by main.py's supervisor for cloud-driven "
                             "calibration on a tabletop demo where the arm cannot fall.")
    parser.add_argument("--no-depth", action="store_true",
                        help="Don't teach the YOLO grab depth with the view-pose snapshot")
    parser.add_argument("--no-arm", action="store_true",
                        help="Skip xArm connection (camera-only test mode)")
    parser.add_argument("--camera-settings", default=cam_settings.DEFAULT_PATH,
                        help="Path to camera_settings.json (shared with main.py).")
    args = parser.parse_args()
    if args.camera is None:  # auto-detect the USB camera, as main.py does
        from core.camera_settings import find_brio_index
        args.camera = find_brio_index(fallback=2)

    if not os.path.exists(HSV_PATH):
        print(f"[calib] ERROR: {HSV_PATH} not found — run calibration/ball_calibrate.py first", file=sys.stderr)
        return 1
    with open(HSV_PATH) as f:
        cfg = json.load(f)
    lower = np.array([cfg["h_min"], cfg["s_min"], cfg["v_min"]], dtype=np.uint8)
    upper = np.array([cfg["h_max"], cfg["s_max"], cfg["v_max"]], dtype=np.uint8)
    print(f"[calib] HSV range: lower={lower.tolist()} upper={upper.tolist()}")

    arm = None
    if not args.no_arm:
        print("[calib] connecting to xArm...")
        arm = xarm.Controller("USB")
        if not args.no_prompt:
            print("\n" + "=" * 60)
            print(" SAFETY: ALL torque is about to drop. Support the arm with your")
            print(" hand before pressing Enter, or it will swing under gravity.")
            print("=" * 60)
            input("[calib] Holding the arm? Press Enter to release torque... ")
        else:
            print("[calib] --no-prompt: skipping safety prompt (cloud-driven launch)")
        try:
            arm.servoOff()
            print("[calib] torque OFF — pose the gripper directly over the ball.")
        except Exception as e:
            print(f"[calib] servoOff failed: {e}")
    else:
        print("[calib] --no-arm: running camera-only (no torque control)")

    state = State()
    state.arm = arm
    depth_teach = None if args.no_depth else _load_depth_teach()
    if depth_teach is not None:
        state.depth_models = (depth_teach[2], depth_teach[3])
    cam_thread = threading.Thread(
        target=camera_loop,
        args=(state, args.camera, args.width, args.height, lower, upper,
              args.camera_settings),
        kwargs={"depth_teach": depth_teach},
        daemon=True,
    )
    cam_thread.start()

    handler = make_handler(state, arm)
    server = ThreadingHTTPServer(("0.0.0.0", args.port), handler)
    print(f"\n[calib] HTTP listening on http://{_local_ip_hint()}:{args.port}/  (Ctrl-C to quit)")

    try:
        while not state.quit:
            server.handle_request()
    except KeyboardInterrupt:
        print("\n[calib] interrupted")
    finally:
        state.quit = True
        cam_thread.join(timeout=2.0)
        if arm is not None:
            try:
                targets = [[sid, int(arm.getPosition(sid))] for sid in ALL_SERVO_IDS]
                arm.setPosition(targets, duration=1500, wait=True)
                print("[calib] re-engaged torque at current pose on exit")
            except Exception as e:
                print(f"[calib] final hold failed: {e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
