#!/bin/bash
# Bootstrap a fresh Arduino VENTUNO Q (Qualcomm Dragonwing IQ-8275, Ubuntu
# 24.04) for the IOTCONNECT XArm demo. Idempotent — safe to re-run if a step
# fails halfway through.
#
# Run as the board's `arduino` user (NOT root) from a shell on the board —
# SSH, the HDMI desktop terminal, or `adb shell` from a PC over the USB-C
# cable. System steps call sudo and ask for the `arduino` password once.
#
# What this DOES handle automatically:
#   - Qualcomm NPU runtime from the board's qcom PPA (qairt-libs: TFLite QNN
#     delegate; qairt-dsp-binaries: Hexagon v75 skel libs in /usr/lib/dsp/cdsp)
#   - udev rule so the `arduino` user can open the xArm over USB HID
#   - YOLO-X + MiDaS-V2 w8a8 models into /etc/models (+ labels into
#     /etc/labels) — the Tria image ships these; the VENTUNO Q does not
#   - miniforge3 + conda env iotc-ventuno-xarm (python 3.11 + opencv)
#   - pip install requirements.txt + requirements-yolo.txt
#   - PointNet model download (model/get_model.sh)
#   - camera_settings.json with the locked-camera recipe (auto_wb=0, etc.)
#   - desktop launchers on the HDMI GNOME desktop (calibrate / demo buttons)
#   - robotarm.service autostart (boots the app in IDLE)
#
# What you STILL have to do MANUALLY (in this order):
#   1. Place the 3 IoTConnect credential files in the project dir BEFORE start.sh
#      can connect to the cloud:
#        - iotcDeviceConfig.json
#        - device-cert.pem
#        - device-pkey.pem
#      If IoTConnect generated the certificate, its zip has cert_<device>.crt
#      and pk_<device>.pem — copy them over (scp / adb push) RENAMED to
#      device-cert.pem / device-pkey.pem. See README "/IOTCONNECT Onboarding".
#      Without these, main.py runs but can't reach IoTConnect.
#
#   2. Connect hardware: xArm USB → VENTUNO Q USB-A, USB camera → the other
#      USB-A (or a hub), arm power adapter (7.5V) → wall, arm switch → ON.
#
#   3. Calibrate. None of these can be automated; they need eyes on the arm:
#        - calibrate_ball                  (cloud cmd; click ball in browser)
#        - calibrate_box                   (cloud cmd; click box in browser)
#        - calibrate_offset                (cloud cmd; pose gripper over ball, snapshot)
#        - teach_scan_pose center/left/right (cloud cmd; release_torque, hand-pose, hold_pose, capture)
#        - teach_drop_pose                 (cloud cmd; pose gripper above box, snapshot)
#
# Once 1-3 are done, ./start.sh --mode pickplace --headless --web-port 8000

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_USER="$(id -un)"
CONDA_ENV="iotc-ventuno-xarm"
MINIFORGE_DIR="$HOME/miniforge3"

# Qualcomm AI Hub w8a8 TFLite models — same files + names the Tria image ships
# in /etc/models (URLs from quic/sample-apps-for-qualcomm-linux
# download_artifacts.sh). Chip-agnostic: the QNN delegate compiles them for
# the v75 HTP at load time.
MODELS_DIR=/etc/models
LABELS_DIR=/etc/labels
YOLOX_URL="https://huggingface.co/qualcomm/Yolo-X/resolve/v0.30.5/Yolo-X_w8a8.tflite"
MIDAS_URL="https://huggingface.co/qualcomm/Midas-V2/resolve/v0.29.1/Midas-V2_w8a8.tflite"
LABELS_URL="https://github.com/quic/sample-apps-for-qualcomm-linux/releases/download/GA1.6-labels/labels.zip"

echo "=================================================================="
echo "  VENTUNO Q board bootstrap — $(date -u +%FT%TZ)"
echo "=================================================================="

# ---- 0. Run as the board user, from the project dir ---------------------
if [ "$(id -u)" = "0" ]; then
    echo "ERROR: run as the board user (arduino), not root — the demo runs as"
    echo "       that user and system steps use sudo."
    exit 1
fi
cd "$PROJECT_DIR"
sudo -v   # ask for the password once, up front

# ---- 1. Qualcomm NPU runtime --------------------------------------------
# The board image has the ubuntu-qcom-iot PPA configured but not the AI
# runtime. qairt-libs provides /usr/lib/libQnnTFLiteDelegate.so (same path
# as on the Tria); qairt-dsp-binaries links the QCS8300-family Hexagon skel
# libs into /usr/lib/dsp/cdsp (ADSP_LIBRARY_PATH, see board_env.sh).
echo
echo "[1/10] Installing Qualcomm NPU runtime (qairt-libs, qairt-dsp-binaries)..."
sudo apt-get update -q
sudo apt-get install -y -q qairt-libs qairt-dsp-binaries v4l-utils unzip curl

# ---- 2. xArm USB access -------------------------------------------------
# The xArm (STMicro 0483:5750) is a USB HID device. Root could always open it
# on the Tria; the `arduino` user needs a udev rule (it's in `dialout`).
echo
echo "[2/10] Installing xArm udev rule..."
sudo tee /etc/udev/rules.d/99-xarm.rules >/dev/null <<'EOF'
# Hiwonder xArm 1S (STMicro 0483:5750) — let the demo user open it over HID
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5750", MODE="0660", GROUP="dialout", TAG+="uaccess"
SUBSYSTEM=="usb", ATTR{idVendor}=="0483", ATTR{idProduct}=="5750", MODE="0660", GROUP="dialout", TAG+="uaccess"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=hidraw --subsystem-match=usb || true

# ---- 3. YOLO-X + MiDaS models + labels ----------------------------------
echo
echo "[3/10] Installing NPU models into $MODELS_DIR ..."
sudo mkdir -p "$MODELS_DIR" "$LABELS_DIR"
fetch_model() {   # url dest
    if [ -s "$2" ]; then
        echo "    $2 already present — skipping"
    else
        echo "    downloading $(basename "$2")"
        sudo curl -fsSL -o "$2.part" "$1" && sudo mv "$2.part" "$2"
    fi
}
fetch_model "$YOLOX_URL" "$MODELS_DIR/yolox_quantized.tflite"
fetch_model "$MIDAS_URL" "$MODELS_DIR/midas_quantized.tflite"
if [ ! -s "$LABELS_DIR/yolox.json" ]; then
    tmp=$(mktemp -d)
    curl -fsSL -o "$tmp/labels.zip" "$LABELS_URL"
    unzip -q -o "$tmp/labels.zip" -d "$tmp"
    sudo cp "$tmp"/labels/* "$LABELS_DIR"/
    rm -rf "$tmp"
    echo "    labels installed in $LABELS_DIR"
else
    echo "    $LABELS_DIR/yolox.json already present — skipping"
fi

# ---- 4. Install miniforge3 if missing -----------------------------------
if [ ! -d "$MINIFORGE_DIR" ]; then
    echo
    echo "[4/10] Installing miniforge3 into $MINIFORGE_DIR ..."
    tmp=$(mktemp -d)
    curl -fsSL -o "$tmp/Miniforge3-Linux-aarch64.sh" \
        "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
    bash "$tmp/Miniforge3-Linux-aarch64.sh" -b -p "$MINIFORGE_DIR"
    rm -rf "$tmp"
else
    echo "[4/10] miniforge3 already at $MINIFORGE_DIR — skipping install"
fi

# Source conda for the rest of this script
source "$MINIFORGE_DIR/etc/profile.d/conda.sh"

# Use absolute paths to the env's binaries throughout. `conda activate` in a
# non-interactive script doesn't always prepend PATH reliably, and Ubuntu's
# system python has no pip.
ENV_BIN="$MINIFORGE_DIR/envs/$CONDA_ENV/bin"
ENV_PYTHON="$ENV_BIN/python3"

# ---- 5. Create conda env if missing -------------------------------------
if ! conda env list | grep -qE "^$CONDA_ENV\s"; then
    echo
    echo "[5/10] Creating conda env $CONDA_ENV..."
    conda create -y -n "$CONDA_ENV" python=3.11
    conda install -y -n "$CONDA_ENV" opencv -c conda-forge
else
    echo "[5/10] conda env $CONDA_ENV exists — skipping create"
fi

# ---- 6. Install pip requirements ----------------------------------------
# `python -m pip` instead of a `pip`/`pip3` binary path — always works as
# long as the env's python is functional.
echo
echo "[6/10] Installing pip requirements..."
"$ENV_PYTHON" -m ensurepip --upgrade >/dev/null 2>&1 || true
"$ENV_PYTHON" -m pip install -q -r requirements.txt
"$ENV_PYTHON" -m pip install -q -r requirements-yolo.txt

# ---- 7. Download PointNet model if missing ------------------------------
if [ ! -f "$PROJECT_DIR/model/point_net_1.pth" ]; then
    echo
    echo "[7/10] Downloading PointNet model..."
    bash model/get_model.sh
else
    echo "[7/10] PointNet model already present — skipping download"
fi

# ---- 8. Drop in camera_settings.json (the locked-camera recipe) --------
if [ ! -f "$PROJECT_DIR/camera_settings.json" ]; then
    echo
    echo "[8/10] Writing camera_settings.json with the locked-camera recipe"
    echo "      (auto_wb=0, auto_exposure=1 manual, exposure=300, gain=80,"
    echo "       wb_temperature=5400, saturation=180 — Brio 100, normal indoor)"
    cat > "$PROJECT_DIR/camera_settings.json" <<'EOF'
{
  "auto_wb": 0,
  "wb_temperature": 5400,
  "auto_exposure": 1,
  "exposure": 300,
  "gain": 80,
  "saturation": 180
}
EOF
else
    echo "[8/10] camera_settings.json already present — skipping (delete it if you want to reset)"
fi

# Make sure all .sh helpers are executable
chmod +x ./*.sh deploy/*.sh 2>/dev/null || true

# ---- 9. Desktop launchers on the HDMI GNOME desktop ---------------------
# Replaces the Tria's Weston panel icons: app-grid entries for ball-calibrate,
# box-calibrate, ball demo, pickplace demo, and a project terminal.
echo
echo "[9/10] Installing desktop launchers..."
APPS_DIR="$HOME/.local/share/applications"
mkdir -p "$APPS_DIR"
make_launcher() {   # file name cmd icon
    cat > "$APPS_DIR/$1.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=$2
Exec=gnome-terminal --working-directory=$PROJECT_DIR -- $PROJECT_DIR/launch_gui.sh $3
Icon=$4
Terminal=false
Categories=Utility;
EOF
}
make_launcher xarm-ball-calibrate "XArm: Calibrate Ball"  calibrate-ball  applications-graphics
make_launcher xarm-box-calibrate  "XArm: Calibrate Box"   calibrate-box   applications-graphics
make_launcher xarm-ball-demo      "XArm: Ball Demo"       ball            media-playback-start
make_launcher xarm-pickplace-demo "XArm: Pick-Place Demo" pickplace       media-playback-start
cat > "$APPS_DIR/xarm-terminal.desktop" <<EOF
[Desktop Entry]
Type=Application
Name=XArm: Terminal
Exec=gnome-terminal --working-directory=$PROJECT_DIR
Icon=utilities-terminal
Terminal=false
Categories=Utility;
EOF
echo "    launchers in $APPS_DIR (search 'XArm' in the app grid)"

# ---- 10. Auto-start service ---------------------------------------------
# Boots the demo in IDLE (arm still, cloud up), recovers from USB drops, and
# waits for DNS + NTP so a cold boot still connects to IoTConnect. Runs
# deploy/arm_watchdog.sh in place, as this user. Enabled (not started) — it
# starts on next boot, after you've connected hardware + dropped in creds.
echo
echo "[10/10] Installing robotarm autostart service..."
sed -e "s|^User=.*|User=$RUN_USER|" \
    -e "s|^Environment=HOME=.*|Environment=HOME=$HOME|" \
    -e "s|^ExecStart=.*|ExecStart=/bin/sh $PROJECT_DIR/deploy/arm_watchdog.sh|" \
    deploy/robotarm.service | sudo tee /etc/systemd/system/robotarm.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable robotarm.service
echo "    enabled robotarm.service — boots the app in IDLE on power-up."
echo "    (start now with: sudo systemctl start robotarm.service)"

echo
echo "=================================================================="
echo "  Bootstrap complete."
echo "=================================================================="
echo
echo "Remaining manual steps:"
echo
echo "  1. Place 3 IoTConnect credential files in $PROJECT_DIR :"
echo "       iotcDeviceConfig.json"
echo "       device-cert.pem"
echo "       device-pkey.pem"
echo "     If IoTConnect generated the certificate, rename while copying:"
echo "       cert_<device>.crt -> device-cert.pem"
echo "       pk_<device>.pem   -> device-pkey.pem"
echo "     (scp or adb push from your PC — see README \"/IOTCONNECT Onboarding\")"
echo
echo "  2. Connect hardware:"
echo "       - xArm USB cable → VENTUNO Q USB-A"
echo "       - USB camera     → the other USB-A (or a hub)"
echo "       - 7.5V arm power adapter plugged into wall + arm"
echo "       - Arm power switch ON"
echo
echo "  3. Verify hardware enumerates:"
echo "       lsusb                      # expect STMicroelectronics 0483:5750 (= xArm) + your camera"
echo "       v4l2-ctl --list-devices    # camera shows as a usb-... device"
echo
echo "  4. Smoke-test the arm (writes only — won't error if reads are flaky):"
echo "       $ENV_PYTHON -c \\"
echo "         \"import xarm; a = xarm.Controller('USB'); a.setPosition([[i,500] for i in range(1,7)], duration=2000, wait=True)\""
echo "     Watch the arm — it should move to neutral pose."
echo
echo "  5. Smoke-test the NPU:"
echo "       source board_env.sh && python yolo_selftest.py"
echo "     Expect: NPU=YES (Hexagon HTP via QNN)"
echo
echo "  6. Start the demo — it AUTO-STARTS on boot in IDLE (arm still, cloud up)"
echo "     via robotarm.service. To start it now without rebooting:"
echo "       sudo systemctl start robotarm.service"
echo "     Then from the IoTConnect dashboard, begin/stop movement with:"
echo "       set_mode mode=yolo-pickplace   (start)"
echo "       set_mode mode=idle             (stop, back to safe idle)"
echo "     Logs: /tmp/yolo.log (app), /tmp/watchdog.log (supervisor)."
echo "     The preview window shows on HDMI only while the arduino user is"
echo "     logged in to the desktop; otherwise watch http://<board-ip>:8080/."
echo
echo "  7. From the IoTConnect dashboard, run calibrations in this order:"
echo "       calibrate_ball       (clicks open browser at http://<board-ip>:8000)"
echo "       calibrate_box"
echo "       calibrate_offset"
echo "       teach_scan_pose name=center  (release_torque -> hand-pose -> hold_pose -> teach)"
echo "       teach_scan_pose name=left"
echo "       teach_scan_pose name=right"
echo "       teach_drop_pose      (pose gripper above box, snapshot)"
echo
echo "  8. Run pickplace:"
echo "       set_mode_pickplace   (cloud cmd)"
echo "     Open http://<board-ip>:8000/ in any browser to watch."
echo
echo "Done. See README.md for details on each command + their args."
