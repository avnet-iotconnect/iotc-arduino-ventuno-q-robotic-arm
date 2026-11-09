# Demos In Depth

How each demo works, how to change it, and how to fix it when it misbehaves.
The [main README](../README.md) covers setup and how to run each demo; this page
is for developers who want to understand or modify the code.

- [How the App Is Put Together](#how-the-app-is-put-together) — shared by all demos
- [Demo 1 — ASL Gesture Control](#demo-1--asl-gesture-control)
- [Demo 2 — HSV Pick-and-Place](#demo-2--hsv-pick-and-place)
- [Demo 3 — YOLO + Depth Pick-and-Place](#demo-3--yolo--depth-pick-and-place)

> [!NOTE]
> Constant values below are the defaults in the code at the time of writing.
> The code is the source of truth — each constant has a comment next to it
> explaining why it has its current value.

---

## How the App Is Put Together

### The supervisor (`main.py`)

`main.py` is one long-running process that owns the arm (USB), the camera, the
/IOTCONNECT connection and the web view. It is always in one of three states:

| State | What runs | How it is entered |
|-------|-----------|-------------------|
| **idle** | Nothing moves; arm and cloud stay connected | `--mode idle` (what `robotarm.service` uses), `set_mode mode=idle`, `stop_demo` |
| **mode** | One vision mode, frame by frame | `--mode <name>`, `set_mode mode=<name>` |
| **calibrator** | A calibration script as a subprocess, with the arm released to it | `calibrate target=<name>` |

Cloud commands arrive on the /IOTCONNECT SDK thread and are queued. Simple
commands (move the arm, change a camera setting) run between frames. Commands
that change state (`set_mode`, `calibrate`, `release_torque` while a mode runs)
set a *pending action*; the camera loop sees it after the current frame, tears
the mode down cleanly, and the supervisor starts the next state. When a
calibrator exits, the supervisor reconnects the arm and returns to the state it
came from.

### The camera loop (`run_mode`)

For every frame:

1. A background thread (`_FreshCamera`) always holds the newest camera frame,
   so a slow frame never builds up a backlog of stale ones.
2. `mode.process_frame(frame, arm)` runs detection and sends arm commands. It
   returns an annotated image.
3. The image goes to the HDMI window (unless headless), the MJPEG web view
   (`--web-port`) and WebRTC (`--webrtc`).
4. Queued cloud commands are processed.

With no `DISPLAY` / `WAYLAND_DISPLAY` (typical over SSH) the loop forces
headless mode automatically. If the camera fails to open, the app drops to idle
instead of exiting, so it stays reachable from the cloud.

### Modes

Every demo is a class in [`modes/`](../modes) with the same small interface
([`modes/base.py`](../modes/base.py)):

| Method | Called |
|--------|--------|
| `setup(arm)` | Once, before the first frame — load models and calibration, move to a start pose |
| `process_frame(frame, arm)` | Every frame — return the annotated image |
| `teardown(arm)` | Once, when the mode stops |
| `get_state()` | For telemetry — the short label sent as `state` |

`make_mode(name)` in [`modes/__init__.py`](../modes/__init__.py) maps a mode name
to its class. To **add a new mode**:

1. Create `modes/my_mode.py` with a class based on `Mode`.
2. Add it to `make_mode()`.
3. Add the name to `VISION_MODE_NAMES` in `main.py` so `set_mode mode=my-mode` accepts it.
4. Optionally add a dashboard button that sends `set_mode` with that name.

Modes call back into `main.py` (`from main import send_telemetry, execute_arm_action`)
inside their methods, not at the top of the file, to avoid a circular import.

### Arm conventions

The Hiwonder xArm 1S has six bus servos. Positions run 0–1000; 500 is the
middle of travel.

| ID | Joint | Notes |
|----|-------|-------|
| 1 | gripper | Larger = more closed |
| 2 | wrist_roll | Keep near 500 while tracking — a rolled wrist rotates the image and confuses the controller |
| 3 | wrist_flex | Larger = camera tilts down |
| 4 | elbow_flex | Direction depends on how the arm was built (see [Approach direction](#approach-direction)) |
| 5 | shoulder_lift | Larger = reach forward / down |
| 6 | shoulder_pan | Rotates the whole arm left / right |

Read servo positions one at a time (`arm.getPosition(sid)`), never with the
batched call: when the batched reply fails to parse it leaves bytes in the USB
buffer, and every later read is garbage until the arm is power-cycled.

### Saved calibration files

All of these are written to the repo root on the board and are git-ignored.
Delete a file to go back to the code defaults.

| File | Written by | Used by |
|------|-----------|---------|
| `ball_color.json` | Ball color calibration (2a) | `ball`, `pickplace`, `calibrate_offset` |
| `box_color.json` | Box color calibration (2c) | `pickplace`, `yolo-pickplace` |
| `scan_poses.json` | `teach_scan_pose` cloud command | All ball modes (overrides the defaults in `modes/ball_follow.py`) |
| `cam_gripper_offset.json` | `calibrate_offset` (2d) | All ball modes |
| `grab_depth.json` | `calibrate_offset` step 1, when the YOLO models are installed | YOLO modes |
| `grab_threshold.json` | `calibrate_offset` step 1, and the grab-gate commands | YOLO modes |
| `box_pose.json` | Pick-and-place, when it locks onto the box | Pick-and-place modes |
| `drop_pose.json` | `teach_drop_pose` cloud command | Pick-and-place modes |
| `camera_settings.json` / `camera_presets.json` | Bootstrap, `camera_setting`, `camera_preset_save` | Everything that opens the camera |

### Telemetry

`send_telemetry()` puts a sample on a queue and a background thread sends it,
so a slow network never stalls the camera loop. Every message carries the six
servo positions, the mode's `state` and system info (`sysInfo_*`). Ball modes
add a `ballTrack` object (see each demo below).

/IOTCONNECT silently drops attributes that the device template does not
declare. If you add a telemetry field, add it to
[`iotconnect/robarmwebrtc-template.json`](../iotconnect/robarmwebrtc-template.json)
and re-import the template.

### Logs

| Log | When |
|-----|------|
| `/tmp/yolo.log` | App started by `robotarm.service` (autostart) |
| `/tmp/watchdog.log` | The autostart supervisor (`deploy/arm_watchdog.sh`) |
| `run.log` (repo root) | App started by hand with `./start.sh` |
| `calibrate.log`, `calibrate_offset.log`, `teach.log` | The launchers in `scripts/` |

Ball modes print a line whenever their state changes (`[ball] GRAB: ...`,
`[ball] scanning -> left`, ...) and timing every 30 frames
(`[ball-perf] detect=… arm=…`).

### Troubleshooting (all demos)

| Symptom | Likely cause / fix |
|---------|--------------------|
| Arm doesn't move at all | Arm power switch off, or the 7.5 V adapter unplugged. `lsusb` should list `0483:5750`. |
| `Camera failed to open /dev/videoN` | Wrong camera index. The message lists the video devices; pass `--camera N`. On the VENTUNO Q, `/dev/video0`, `1`, `32`, `33` are not cameras. |
| Two programs fighting over the arm, jerky moves | The autostart app is still running. `touch /tmp/demo_watchdog_off && pkill -f 'python -u main.py'` before running anything by hand. |
| Arm stops responding after a USB glitch | The watchdog relaunches the app when the arm re-enumerates. If running by hand, restart it. |
| Cloud commands do nothing | Check the log for `Unknown command`. The command must be declared on the device template, and the app must be connected (`iotcDeviceConfig.json` + certificates in the repo root). |
| Dashboard field always empty | The attribute is not declared on the device template. |

---

## Demo 1 — ASL Gesture Control

Code: [`modes/asl.py`](../modes/asl.py), model: [`core/point_net.py`](../core/point_net.py).

### How it works

Each frame:

1. The frame is mirrored, so the preview behaves like a mirror.
2. **MediaPipe Hands** finds each hand and its 21 landmarks (x, y, z), and
   whether it is a left or right hand. Detection and tracking confidence are
   both 0.5.
3. The landmarks are normalized: x and y are scaled to 0–1 within the hand's
   own bounding box, so distance from the camera does not matter. Right-hand x
   coordinates are flipped, so one model handles both hands.
4. **PointNet** (a small PyTorch network for point clouds) classifies the 21
   points as one of 24 static ASL letters (A–Y, without J and Z, which need
   motion). The highest-scoring letter wins; there is no confidence threshold.
5. If that (hand, letter) pair maps to an action, the arm moves.

All of this runs on the CPU. Telemetry is sent every 2 seconds and after each
action.

### When a gesture fires

- An action fires only when the recognized (hand, letter) **differs from the
  last one recognized**. Holding a sign does not repeat it.
- Hiding your hand does **not** reset this. To repeat an action, show any
  other letter first (an unmapped letter works), then sign it again.
- After an action, one frame is skipped before the next can fire.
- Arm moves block the camera loop until they finish (1–1.5 s), so the preview
  pauses during a move.
- With two hands in view, the hand processed last in that frame counts. Sign
  with one hand at a time.

### Startup sequence

1. The app connects to the arm and /IOTCONNECT.
2. PointNet loads from `model/point_net_1.pth` (~2–3 seconds).
3. The camera loop starts. The overlay stays blank until a hand appears.

### On-screen overlay

- `LEFT=<letter>` (top-left, green) / `RIGHT=<letter>` (top-right, red) — the
  recognized letter for each hand
- `[<action>]` under the letter — the action that letter maps to
- The hand skeleton drawn over the detected landmarks

### Telemetry — ASL

| Field | Description |
|-------|-------------|
| `state` | `"ASL-Gesture"` (constant) |
| `gripper`, `wrist_roll`, `wrist_flex`, `elbow_flex`, `shoulder_lift`, `shoulder_pan` | Servo positions (0–1000) |
| `sysInfo_cpu` | CPU usage + temperature |
| `sysInfo_memory` | RAM usage + temperature |
| `sysInfo_storage` | Disk usage |
| `sysInfo_gpu` | Adreno GPU % (near zero — the workload is on the CPU) |

### How to modify it

| Change | Where |
|--------|-------|
| Which letter does what | `LEFT_GESTURE_TO_ACTION` / `RIGHT_GESTURE_TO_ACTION` in `modes/asl.py`. Values are action names from `execute_arm_action()` in `main.py`: `advance`, `backup`, `left`, `right`, `up`, `down`, `home`, `open_gripper`, `close_gripper`, `wrist_roll_cw`, `wrist_roll_ccw`, `wrist_flex_up`, `wrist_flex_down`, and the scripted `demo_*` moves. |
| Arm moving the wrong way | Swap the action names in the mapping (e.g. `'advance'` ↔ `'backup'`). |
| How far each move goes | `execute_arm_action()` in `main.py` — each move is ±100 servo units over 1000–1500 ms. |
| Add a new action | Add it to `ACTION_LABELS` and `execute_arm_action()` in `main.py`, then map a letter to it. To trigger it from the cloud too, add it to `IOTC_COMMAND_TO_ACTION` and the device template. |
| Hand-detection sensitivity | `min_detection_confidence` / `min_tracking_confidence` in `ASLMode.setup()`. |
| Use a different PointNet model | `model_dir` / `model_name` in `ASLMode.__init__`. The class order in `CHAR2INT` must match the order the model was trained with. |

### Troubleshooting — ASL

| Symptom | Likely cause / fix |
|---------|--------------------|
| `PointNet module not available` or the model file is missing | Run `model/get-model.sh` (step 3 of First-Time Setup). The file must be ~38–42 MB. |
| No skeleton drawn over the hand | Hand too small, too far, or backlit. Fill ¼–⅓ of the frame, light the hand from the front. |
| Letters flicker between two signs | Similar shapes (e.g. A/S/T, M/N) are hard for PointNet. Hold the sign clearly, palm toward the camera. Map actions to distinct letters if you change the mapping. |
| The same action won't fire twice | Expected — show a different letter in between (see [When a gesture fires](#when-a-gesture-fires)). |
| Arm moves the opposite way | Mount orientation; swap the mapping (see above). |
| Preview freezes for a second | Expected while the arm moves. |

---

## Demo 2 — HSV Pick-and-Place

Code: [`modes/ball_follow.py`](../modes/ball_follow.py) (`--mode ball`),
[`modes/pickplace.py`](../modes/pickplace.py) (`--mode pickplace`).

### How `ball` mode works

This is *eye-in-hand visual servoing*: the camera rides on the arm, and the arm
moves until the ball appears where it appeared during calibration.

Each frame:

1. **Read the arm.** All six servo positions, one at a time.
2. **Find the ball.** Convert to HSV, keep pixels inside the calibrated range
   (`ball_color.json`), clean the mask (open 5×5, close 9×9), and take the
   largest blob that is round enough: area ≥ `MIN_CONTOUR_AREA` (200 px) and
   filling ≥ `MIN_FILL_RATIO` (65%) of its enclosing circle. Logos, shadows and
   fabric edges usually fill well under half, so they're rejected.
3. **Work out the error.** The aim point is the image center plus the
   calibrated camera-gripper offset (the pixel where the ball sat at the view
   pose). `pan_err` / `tilt_err` are the pixel distance from the ball to the
   aim point; `radius_err` is how far the ball's apparent radius is from the
   taught radius.
4. **Decide.**
   - Ball centered (left/right within 60 px, up/down within 30 px) **and** at
     grab distance (radius within ±15% of the taught one) → **grab**.
   - Otherwise send one small correction: shoulder_pan for left/right,
     wrist_flex for up/down, and — once the ball is roughly centered (within
     60 px) but still too small — a step of shoulder_lift + elbow_flex toward
     it.
5. **Move.** One `setPosition` per frame, 220 ms long, without waiting. The
   servos retarget smoothly when the next frame sends a new command.

The controller is proportional (step = gain × pixel error), with two
adjustments that matter on these servos:

- **Minimum step.** Bus servos ignore small commands because of static
  friction. Any correction is raised to at least `MIN_TRIM_STEP` (14) for the
  wrist and `MIN_TRIM_STEP_PAN` (18) for the pan, which carries the whole arm.
- **Maximum step.** No single move exceeds `MAX_STEP` (25), so a large error
  can't fling the arm.

The deadbands are sized from these. One pan step moves the image about 126 px, so a narrow
left/right deadband makes the arm overshoot back and forth.

### State machine — `ball`

| State | What it does | Next |
|-------|--------------|------|
| `IDLE` | Moves to the first scan pose at startup | First frame → `SCANNING` / `TRACKING` |
| `SCANNING` | No ball. After `NO_BALL_GRACE_FRAMES` (30) frames without a ball, cycles through the scan poses (2.5 s move + 1.5 s look at each) | Ball seen → `TRACKING` |
| `TRACKING` | Centers on the ball and reaches toward it. The in-progress scan move is halted the moment the ball appears, so the arm doesn't sweep past it | Centered + at distance → grab |
| `GRABBING` | Runs the grab sequence (below) | Holding → `HOLDING`; missed → `TRACKING` |
| `HOLDING` | Ball held at home pose; tracking paused | Gripper opened by ≥ 40 units (`open_gripper`) → resumes |

`PREDICTING` (follow the ball's last motion for a few frames after losing it)
exists but is switched off (`ENABLE_PREDICTION = False`).

### The grab sequence

1. Open the jaws fully. If suction-cup tips are stuck together, squeeze
   briefly and snap open at full speed, up to 4 times.
2. **Reach** (only if a grab pose was taught in `calibrate_offset` step 2):
   repeat the taught move from the view pose to the grab pose. It is replayed
   as a move *of the gripper* ([`core/arm_kinematics.py`](../core/arm_kinematics.py)),
   not as raw joint changes, so it still lands right when tracking ended with
   the arm in a slightly different shape than during calibration.
3. Close fast (250 ms) so the servo is at full force when it meets the ball,
   then wait until the jaw position stops changing (up to 3 s) and dwell 0.5 s.
4. **Empty check.** Jaws closed to within 10 units of fully shut → nothing
   was caught. Re-open, go back to the view pose, keep tracking.
5. Lift (shoulder_lift 300, elbow_flex 350).
6. **Slip check.** Jaws at 900 or more after the lift → the ball fell out.
   Re-open, keep tracking.
7. Go home with the gripper still closed.

The gripper stays commanded fully closed for the whole carry. Commanding a
gentler hold makes the servo relax for a moment and the ball shifts.

### Scan poses

The arm searches by cycling through scan poses. They come from, in order of
priority:

1. `scan_poses.json`, written by the `teach_scan_pose name=<n>` cloud command.
   Labels are whatever names you used; they're visited in the order taught.
2. `SCAN_POSES` in `modes/ball_follow.py`. `calibration/teach_pose.py` rewrites
   this list (near and far rows plus an interpolated middle row).

Scan poses are always saved with the gripper open, because the grab only ever
closes from wherever the gripper is. The pan range allowed during tracking is
widened automatically to cover every scan pose, so a ball found at the edge of
the search can still be followed.

### Approach direction

Arms are not all assembled the same way: on some builds the elbow bends the
other way, and "reach forward" means *decreasing* shoulder_lift and elbow_flex.
`ApproachPlan` in `modes/ball_follow.py` works this out per arm:

1. From the taught grab reach, if there is one: its shoulder/elbow move *is*
   "toward the ball" for this arm.
2. Otherwise from the scan pose: an elbow above 500 means a mirror-image build.

It also sets limits so a ball that never reaches grab size can't drive the
gripper into the table: the shoulder may go at most `APPROACH_REACH` (250)
past the view pose, and the elbow stops at straight (500) or `REACH_MARGIN`
(100) past the view pose. When both joints are at their limits the log prints
`OUT-OF-REACH`.

### How `pickplace` mode works

`pickplace` wraps `ball` mode: it runs the same ball logic, and when the
gripper closes on a ball it carries the ball to the box.

| State | What it does |
|-------|--------------|
| `BOX_SEEK` | Cycle the scan poses looking for the box |
| `BOX_LOCKING` | Center the box in view (pan + wrist only, no descent; wrist kept pointing down, 750–1000). When centered within 200 px, save the arm pose to `box_pose.json` |
| `BOX_VERIFY` | Move to the saved box pose and check the box is still there: still centered → `BALL_PHASE`; visible but off-center → `BOX_LOCKING`; not seen within 8 frames → `BOX_SEEK` |
| `BALL_PHASE` | Ball mode, except balls inside the box are ignored |
| `TRANSPORT` | Move to the drop pose |
| `DROPPING` | Open the gripper |
| `HOMING` | Go home with the jaws open, then `BOX_VERIFY` (or straight to `BALL_PHASE`, see below) |

Details:

- **The box** is the largest region of box color whose outline simplifies to a
  4–6 sided polygon.
- **Ignoring balls already in the box.** Each frame, the box-colored region is
  filled in (convex hull) and grown by 30 px. Ball candidates whose center falls
  inside are skipped. If the box flickers out of view, the last outline is kept
  for 30 frames.
- **Drop pose.** With `drop_pose.json` (taught with `teach_drop_pose`), the arm
  goes straight there, and box seek / lock / verify are skipped entirely — the
  box color is then only used to ignore balls already in it. Without it, the
  arm drops from the box lock pose with shoulder_lift raised by
  `DROP_LIFT_OFFSET` (50).
- The cycle repeats automatically after each drop.

### What each calibration stores

| Calibration | File | Contents |
|-------------|------|----------|
| Ball / box color | `ball_color.json`, `box_color.json` | HSV min/max (each click's 7×7 median, padded by H ±15, S ±25, V ±30) and the raw samples. For low-saturation (white/grey) targets hue is opened to the full range. |
| Scan poses | `scan_poses.json` | Named full-arm poses |
| Camera-gripper offset | `cam_gripper_offset.json` | `cam_gripper_offset_x/y`: where the ball sits in the image (relative to center) at the view pose. `target_radius_px`: its apparent radius there. `view_pose`: the arm pose. `grab_reach`: the joint change from view pose to grab pose (step 2). Redoing step 1 clears the reach. |
| Box lock | `box_pose.json` | Arm pose with the box centered |
| Drop pose | `drop_pose.json` | Arm pose to release the ball from |

### Tuning constants (`modes/ball_follow.py`)

| Constant | Default | Notes |
|----------|---------|-------|
| `PAN_GAIN`, `TILT_GAIN` | 0.04, 0.10 | Servo units per pixel of error. Tilt is higher because wrist_flex fights gravity at extended poses. |
| `PAN_DIR`, `TILT_DIR` | 1, −1 | Sign flips. Flip one if the arm moves *away* from the ball on that axis. If the wrist is pinned at its limit, re-teach the scan pose with wrist_flex ≤ ~900 instead of flipping. |
| `MIN_TRIM_STEP` | 14 | Smallest wrist_flex correction. Bus servos ignore smaller commands. |
| `MIN_TRIM_STEP_PAN` | 18 | Smallest shoulder_pan correction (it carries the whole arm). |
| `MAX_STEP` | 25 | Largest single-frame change for any joint. |
| `MOVE_DURATION_MS` | 220 | Duration of each per-frame move. Too short and small moves are ignored; too long and the loop slows down. |
| `CENTER_DEADBAND_PX` | 60 | Left/right error allowed before a grab. Too narrow and pan overshoots back and forth (one pan step ≈ 126 px). |
| `GRAB_TILT_DEADBAND_PX` | 30 | Up/down error allowed before a grab. Tighter than left/right because a ball high in the image is farther away than taught. |
| `APPROACH_DEADBAND_PX` | 60 | How centered the ball must be before reaching toward it. |
| `APPROACH_STEP` | 15 | shoulder_lift change per reach step. |
| `ELBOW_REACH_RATIO` | 0.6 | Elbow change per unit of shoulder_lift while reaching (when no grab reach was taught). |
| `TARGET_RADIUS_PX` | 220 | Fallback grab radius. Normally replaced by the calibrated `target_radius_px`. |
| `RADIUS_TOLERANCE_FRAC` | 0.15 | Grab radius tolerance as a fraction of the target (at least 5 px). |
| `MIN_CONTOUR_AREA`, `MIN_FILL_RATIO` | 200, 0.65 | Ball blob filters. |
| `PAN_MIN`, `PAN_MAX` | 90, 860 | Minimum pan range while tracking (widened to cover the scan poses). |
| `APPROACH_REACH`, `REACH_MARGIN` | 250, 100 | How far the reach may go past the view pose (shoulder, elbow). |
| `NO_BALL_GRACE_FRAMES` | 30 | Frames without a ball before scanning resumes (~5 s). Keeps brief detection drop-outs from sending the arm back to scanning. |
| `SCAN_DWELL_S`, `SCAN_MOVE_MS` | 1.5, 2500 | Time spent looking at each scan pose / moving between them. |
| `GRAB_REACH_MS` | 1200 | Duration of the taught reach to the grab pose. |
| `GRIPPER_CLOSE_MS` | 250 | Close speed. |
| `GRIPPER_EMPTY_POS` | 900 | Jaws at or beyond this after the lift = ball lost. |
| `GRIPPER_UNSTICK_TRIES` | 4 | Snap-open attempts for stuck suction-cup tips. |
| `TELEMETRY_INTERVAL_S` | 5.0 | How often `ballTrack` telemetry is sent. |

Pick-and-place adds its own in `modes/pickplace.py`: `BOX_CENTER_DEADBAND_PX`
(200), `BOX_WRIST_FLEX_MIN/MAX` (750/1000), `DROP_LIFT_OFFSET` (50),
`BALL_BOX_EXCLUSION_DILATE_PX` (30), `BALL_BOX_HULL_STALE_FRAMES` (30),
`BOX_VERIFY_GRACE_FRAMES` (8), `TRANSPORT_DURATION_MS` (2000).

> [!WARNING]
> `PAN_GAIN`, `TILT_GAIN` and `MAX_STEP` act per frame, so they scale with the
> frame rate. If you change resolution or turn off the preview, expect to
> re-tune.

### Telemetry — HSV

Sent every 5 s and after each grab. `state` is the current state; `ballTrack`
contains:

| Field | Description |
|-------|-------------|
| `state` | Ball-mode state (`SCANNING` / `TRACKING` / `HOLDING` / …) |
| `ball_x`, `ball_y`, `ball_r` | Last ball position and radius in pixels |
| `pan_err`, `tilt_err` | Distance from the aim point in pixels |
| `radius_err` | Taught radius minus current radius |
| `d_pan`, `d_tilt`, `d_lift`, `d_elbow` | Last commanded servo changes |
| `no_ball_frames` | Consecutive frames without a ball |
| `hold_target` | Gripper position where it stalled on the ball (0 = not holding) |
| `is_prediction`, `pred_frames_left`, `velocity_x`, `velocity_y` | Prediction state (unused while prediction is off) |

### Troubleshooting — HSV

| Symptom | Likely cause / fix |
|---------|--------------------|
| `ball_color.json not found` | Run ball color calibration (2a). |
| `box_color.json not found` | Run box color calibration (2c). |
| Ball not detected, or other objects detected | Redo color calibration under the demo lighting. Lock the camera: `camera_setting wb_temperature auto`, then recalibrate. Remove ball-colored objects from the table. |
| Arm hunts back and forth around the ball | Deadband too narrow for the pan step, or gains too high. Widen `CENTER_DEADBAND_PX` or lower `PAN_GAIN`. |
| Ball sits just outside center, arm doesn't move | The step is below the servo's friction floor. Raise `MIN_TRIM_STEP` (wrist) or `MIN_TRIM_STEP_PAN`. |
| Arm moves away from the ball | Flip `PAN_DIR` or `TILT_DIR` for that axis. |
| Wrist won't tilt any further toward the ball | wrist_flex is at its limit. Re-teach that scan pose with the wrist less bent (≤ ~900). |
| Log shows `OUT-OF-REACH` | The ball is farther than the reach limits allow. Move the ball closer, re-teach the scan pose closer to it, or redo `calibrate_offset`. |
| Grabs too early / too late | Redo `calibrate_offset` step 1 — it sets the grab radius and aim point. |
| Jaws close beside the ball | Aim point is off. Redo `calibrate_offset`, holding the ball fully in view at step 1. If the camera can't see the ball between the jaws, also do step 2. |
| `nothing in the jaws; back to tracking` repeatedly | Grab position is off (see above), or the ball rolls away as the jaws close. Suction-cup tips help a lot (README → Hardware Assembly). |
| `jaws still stuck` | Suction cups sealed together; pull them apart by hand. |
| Pick-and-place goes back to the ball already in the box | Box color not detected well enough to exclude it. Redo box color calibration (2c). |
| Ball dropped beside the box | Teach a drop pose (`teach_drop_pose`), or adjust `DROP_LIFT_OFFSET`. |
| Box lock never completes | The wrist hits its 750–1000 range before the box is centered. Move the box closer to a scan pose, or teach a drop pose to skip box locking. |

---

## Demo 3 — YOLO + Depth Pick-and-Place

Code: [`modes/yolo_pickplace.py`](../modes/yolo_pickplace.py),
[`detectors/yolo_detector.py`](../detectors/yolo_detector.py),
[`detectors/depth_detector.py`](../detectors/depth_detector.py).

### What changes from Demo 2

The YOLO modes are subclasses of the Demo 2 modes. Only two things change:

1. **Ball detection** uses a YOLO object detector on the Hexagon NPU instead of
   the HSV color filter. The largest detection (the nearest ball) is tracked.
2. **The "close enough to grab" check** can use a depth estimate from MiDaS,
   also on the NPU, in addition to (or instead of) the ball's apparent size.

Scanning, the control loop, the grab sequence, box handling, transport and
drop are inherited unchanged. The box is still found by color, so
`yolo-pickplace` still needs `box_color.json`.

### Models

| Model | Default | Notes |
|-------|---------|-------|
| Ball detector | `model/ball_best.tflite` if present (confidence 0.5), else the board's stock `/etc/models/yolox_quantized.tflite` (confidence 0.25) | The stock YOLO-X model is trained on COCO and is filtered to class 32, "sports ball". A custom model is assumed to be single-class. See [custom-npu-detector.md](custom-npu-detector.md) to train one. |
| Depth | `/etc/models/midas_quantized.tflite` | Optional. Without it, the depth check is off and the grab uses apparent size only. |

Override the detector with environment variables when using `start.sh`:

```bash
YOLO_MODEL=/path/to/model.tflite YOLO_CONF=0.4 ./start.sh --mode yolo-pickplace --web-port 8080
```

Both models run through the QNN TFLite delegate on the Hexagon NPU. If the
delegate can't load, they fall back to the CPU (much slower) and the startup
log says `NPU=no (CPU TFLite)`. Set `YOLO_VERBOSE=1` to see the delegate's own
error output.

### Depth

MiDaS outputs *relative inverse depth*: a number per pixel where **higher means
closer**. The units are arbitrary and change with the scene and lighting, which
is why the grab depth has to be re-taught at each site.

`D` is the median depth in a patch at the ball's center (patch radius = a third
of the ball's radius, so it stays on the ball). Single-frame MiDaS readings are
noisy (about ±20), so the check uses a smoothed value (exponential moving
average, `D_EWMA_ALPHA` = 0.25, roughly a 4-frame average).

### The grab check

A grab fires when, on a real detection:

1. the ball is centered (same limits as Demo 2), **and**
2. the distance check has passed for `DEPTH_SETTLE_FRAMES` (3) frames in a row.

The distance check has three modes (default `both`):

| Mode | Passes when | Keeps reaching while |
|------|-------------|----------------------|
| `depth` | `D` ≥ depth threshold | `D` is below it |
| `radius` | ball radius ≥ radius floor | radius is below it |
| `both` | both pass | either is below |

Both are minimums with no upper limit: a ball closer than taught still counts.

- **Depth threshold:** default 750. `calibrate_offset` step 1 sets it to the
  taught depth minus a noise margin (the larger of 20 and 1.5 × the spread
  measured during the snapshot).
- **Radius floor:** the taught ball radius (`ball_r_at_grab` in
  `grab_depth.json`) minus 15%. Falls back to `TARGET_RADIUS_PX`.
- Without a depth model, `radius` is used regardless of the setting.

Pan corrections are also spaced at least `PAN_COOLDOWN_MS` (250 ms) apart. At
YOLO frame rates, sending a new pan command every frame retargets the servo
before it finishes moving and the ball swings back and forth.

### Teaching the grab depth

Step 1 of `calibrate_offset` (with the YOLO and MiDaS models installed) loads
the same detector the YOLO modes use, records depth and ball radius over the
recent frames, and writes:

- `grab_depth.json` — median depth, spread, median radius, frame count, and the
  model names used
- `grab_threshold.json` — the new depth threshold

The snapshot is refused if fewer than 5 frames saw the ball, or if the depth
didn't vary at all (this means torque was on and the arm was perfectly still;
hold the arm by hand with torque off).

### Live controls

These change the grab check without restarting the mode. Changes are saved to
`grab_threshold.json`.

| Command | Effect |
|---------|--------|
| `calibrate grab_depth show` | Show the current check: mode, depth threshold, radius floor |
| `calibrate grab_depth <D>` | Set the depth threshold (e.g. `calibrate grab_depth 650`) |
| `set_grab_gate <depth\|radius\|both>` | Choose which signals must pass |
| `set_grab_threshold <D>` | Same as `calibrate grab_depth <D>` |
| `set_grab_radius <px\|auto>` | Set the radius target, or `auto` to use the taught one |
| `grab_threshold_show` / `grab_threshold_reset` | Show / restore the defaults |

> [!NOTE]
> These are declared on the bundled device template. If your device uses a
> template imported before they were added, re-import
> [`iotconnect/robarmwebrtc-template.json`](../iotconnect/robarmwebrtc-template.json).

### Running the standalone app

`./scripts/start-yolo.sh` runs [`tools/yolo_pickplace.py`](../tools/yolo_pickplace.py),
a standalone version without /IOTCONNECT, with every option on the command line:

```bash
./scripts/start-yolo.sh --model model/ball_best.tflite --depth --conf 0.7 \
                        --camera N --headless --web-port 8080
```

| Option | Default |
|--------|---------|
| `--mode` | `pickplace` (or `ball`) |
| `--model` | `/etc/models/yolox_quantized.tflite` |
| `--conf` | 0.25 for stock YOLO-X, 0.7 for a custom model |
| `--depth` / `--depth-model` | off / `/etc/models/midas_quantized.tflite` |
| `--cpu` | Force CPU instead of the NPU |
| `--camera`, `--headless`, `--web-port` | Same as `main.py` |

### Defaults when started with `start.sh`

| Setting | Value |
|---------|-------|
| Model | `model/ball_best.tflite`, else `/etc/models/yolox_quantized.tflite` |
| Confidence | 0.5 for the custom model, 0.25 for stock YOLO-X |
| Depth | On if `/etc/models/midas_quantized.tflite` exists |
| Grab check | `both`, with the taught values from `grab_depth.json` / `grab_threshold.json` |

### Expected startup log

```
[yolo] model=ball_best.tflite format=yolov8 ... NPU=YES (Hexagon HTP via QNN)
[depth] model=midas_quantized.tflite ... NPU=YES (Hexagon HTP via QNN)
[yolo-ball] taught reference loaded: D_grab=860 +/-30, ball_r_at_grab=152px
[yolo-ball] grab gate ARMED: mode=both  D >= 829  r >= auto (taught)
```

### Live overlay

```
r=152 dx=+4 dy=-12                          ← ball radius and distance from the aim point
center=OK  settle 2/3  [both] D+>=829 r+>=129
                                            ← centered? frames passed; mode; each check
                                              (+ = passing, - = not yet) and its minimum
D=860 (raw 884)  higher=closer              ← smoothed and raw depth at the ball
NPU: yolo=7.2ms  depth=4.8ms  14 fps        ← live inference time and frame rate
```

`gate=NO` replaces `settle n/3` while the distance check is failing.

### Tuning constants (`modes/yolo_pickplace.py`)

| Constant | Default | Notes |
|----------|---------|-------|
| `PAN_COOLDOWN_MS` | 250 | Minimum time between pan corrections. |
| `CENTERED_LATCH_FRAMES` | 0 | Keep "centered" true for N frames after it first passes. Off by default — it fired grabs too early in testing. |
| `DEPTH_SETTLE_FRAMES` | 3 | Frames in a row the distance check must pass before grabbing. |
| `D_EWMA_ALPHA` | 0.25 | Depth smoothing. Higher reacts faster but is noisier. |
| `DEPTH_TOL_FLOOR`, `DEPTH_TOL_MULT` | 20, 1.5 | Noise margin below the taught depth: max(floor, mult × spread). In `core/grab_threshold_store.py`. |

All Demo 2 constants (gains, deadbands, reach limits, gripper behavior) apply
too.

### Telemetry — YOLO

All the Demo 2 `ballTrack` fields, plus:

| Field | Description |
|-------|-------------|
| `yolo_ms`, `depth_ms` | Live inference time per model on the NPU |
| `npu_fps` | Frame rate of the whole loop |
| `depth_at_ball`, `depth_at_ball_smoothed` | Raw and smoothed `D` |
| `depth_settle` | Frames in a row the distance check has passed |
| `D_threshold`, `gate_mode`, `r_target` | Current grab-check settings |
| `D_grab`, `D_tolerance` | Taught depth and the noise margin |

`sysInfo_gpu` stays near zero: the work runs on the Hexagon NPU, not the Adreno
GPU. NPU activity shows up in `yolo_ms` and `depth_ms`.

The README's per-model timings (~7 ms YOLO, ~5 ms MiDaS) were measured on the
Tria QCS6490 build of this demo. Check the overlay or telemetry for the numbers
on your board.

### Troubleshooting — YOLO

| Symptom | Likely cause / fix |
|---------|--------------------|
| `NPU=no (CPU TFLite)` | The QNN delegate didn't load. Check `qairt-libs` and `qairt-dsp-binaries` are installed (re-run the bootstrap) and launch through `start.sh` so `ADSP_LIBRARY_PATH` is set. Test with `source scripts/board-env.sh && python tools/diagnostics/yolo_selftest.py`. |
| Model file not found | The bootstrap downloads the stock models to `/etc/models`. Re-run it. |
| No detections | Stock YOLO-X only knows generic "sports ball" and may miss unusual balls. Lower the confidence (`YOLO_CONF=0.15`) or train a custom model. `tools/diagnostics/yolo_diag_stream.py` shows raw detections from the running app's stream. |
| False detections | Raise `YOLO_CONF`, or train a custom model with your background in the images. |
| `calibrate_offset` won't start | It needs `ball_color.json`; run ball color calibration (2a) first. |
| Overlay shows `D-` and the arm stops short | The depth threshold is too high for this scene. Redo `calibrate_offset` step 1, or lower it with `calibrate grab_depth <D>`. |
| Overlay shows `r-` and the arm stops short | Radius floor too high. Redo step 1, or `set_grab_radius <px>`. |
| Grabs fire too early | Threshold too low; redo step 1, or raise it. Using `both` instead of `depth` helps. |
| Grabs worked yesterday, not today | Depth changes with room and light. Redo `calibrate_offset` at each site and after lighting changes. |
| Grabbing got worse after switching models | `grab_depth.json` was taught with the old model. Redo `calibrate_offset` step 1. |
| Depth bounces around | MiDaS is noisy at INT8. Use `both` or `radius`, or lower `D_EWMA_ALPHA`. `tools/diagnostics/depth_monitor.py` measures the noise. |
| Picks up the ball already in the box | Box color not detected; redo box color calibration (2c). |
