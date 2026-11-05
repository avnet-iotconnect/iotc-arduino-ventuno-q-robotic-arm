# SPDX-License-Identifier: MIT
# Copyright (C) 2026 Avnet
"""Browser server for teach_grab_depth.py's --web-driven mode.

Same shape as capture_web.py (MJPEG stream + on-page buttons + /action
endpoints) but with the four buttons the grab-depth teach needs:

  - Release torque         servoOff() so the operator can hand-pose
  - Hold torque            re-enable at current pose
  - Snapshot D_grab        capture N frames + save grab_depth.json
  - Quit                   exit cleanly, leaving torque state as-is

Kept separate from web_view.py (view-only, shared by the demos) so the
teach-only controls don't leak into the demo path.
"""

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>XArm - Teach Grab Depth</title>
<style>
  body { font: 15px sans-serif; margin: 1em; background: #111; color: #eee; }
  #stream { display: block; max-width: 100%; border: 1px solid #333; }
  #status { padding: .5em 0; font-weight: bold; color: #6cf; }
  .row { margin: .5em 0; }
  button { font-size: 16px; padding: .6em 1.1em; margin-right: .5em; border: 0;
           border-radius: 6px; cursor: pointer; color: #fff; }
  .rel   { background: #b03030; }
  .hold  { background: #305fb0; }
  .snap  { background: #2a8a2a; }
  .quit  { background: #555; }
  ol { max-width: 60em; line-height: 1.5; padding-left: 1.4em; }
  .note { max-width: 60em; color: #aaa; }
</style></head>
<body>
<h2>XArm - teach grab depth</h2>
<p class="note">Teaches the YOLO demo how close the ball looks at the moment the arm
should decide to grab, and sets the grab threshold from it. One snapshot.</p>
<ol>
  <li><b>Support the arm</b> with one hand, then click <b>1. Release torque</b>.</li>
  <li>Pose the arm by hand where it should decide to grab, with the ball
      <b>fully in view</b> (green circle, live <b>D</b> value below).
      If you taught a grab move in <code>calibrate_offset</code>, this is the
      <b>view pose</b> from its step 1, not the grab pose.</li>
  <li><b>Keep holding the arm</b> (torque off) and click <b>2. Snapshot</b>.
      Wait about 2 seconds; the status line should end with
      <code>grab gate now D &gt;= ...</code>.</li>
  <li>Click <b>3. Hold torque</b> to lock the arm, then <b>4. Quit</b>
      (closing this tab does not stop the calibrator).</li>
</ol>
<div id="status">connecting...</div>
<div class="row">
  <button class="rel"  onclick="release()">1. Release torque</button>
  <button class="snap" onclick="act('snap')">2. Snapshot</button>
  <button class="hold" onclick="act('hold')">3. Hold torque</button>
  <button class="quit" onclick="act('quit')">4. Quit</button>
</div>
<img id="stream" src="/stream" alt="(camera stream)">
<script>
async function act(cmd) {
  try { const s = await (await fetch('/action?cmd=' + cmd)).json(); render(s); }
  catch (e) {}
}
function release() {
  if (confirm('SUPPORT THE ARM FIRST. Releasing torque lets it fall under gravity. Continue?'))
    act('release');
}
function render(s) {
  document.getElementById('status').textContent =
    `torque: ${s.torque ? 'ON' : 'OFF'}   ball: ${s.ball || 'not detected'}` +
    `   D: ${s.D || '-'}   ${s.msg || ''}`;
}
async function poll() {
  try { render(await (await fetch('/state')).json()); } catch (e) {}
}
setInterval(poll, 1000); poll();
</script>
</body></html>
"""


class TeachGrabWebView:
    def __init__(self, port, on_action, jpeg_quality=80):
        self._lock = threading.Lock()
        self._frame_jpeg = b""
        self._status = {"torque": True, "ball": "", "D": "", "msg": ""}
        self._on_action = on_action
        self._jpeg_quality = jpeg_quality
        self._quit = False
        self._server = ThreadingHTTPServer(("0.0.0.0", port), self._make_handler())
        self._thread = threading.Thread(target=self._server.serve_forever,
                                        kwargs={"poll_interval": 0.5}, daemon=True)
        self._thread.start()
        self.port = port

    def publish(self, bgr_frame, status=None, **ignored):
        """Tolerant of the demo WebView's (state=, mode=, fps_hint=) kwargs so
        teach_grab_depth.py's existing background loop can drive this view
        unchanged. ``status`` is optional — None keeps the previous one
        (action handlers mutate it directly via update_status)."""
        ok, buf = cv2.imencode(".jpg", bgr_frame,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality])
        if not ok:
            return
        with self._lock:
            self._frame_jpeg = buf.tobytes()
            if status is not None:
                self._status = dict(status)

    def update_status(self, **fields):
        """Merge new fields into the published /state without touching the frame."""
        with self._lock:
            self._status.update(fields)

    def stop(self):
        self._quit = True
        try:
            self._server.shutdown(); self._server.server_close()
        except Exception:
            pass

    def url_hint(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80)); ip = s.getsockname()[0]; s.close()
        except OSError:
            ip = "0.0.0.0"
        return f"http://{ip}:{self.port}/"

    def _make_handler(view):
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *a):
                return

            def _json(self, obj):
                body = json.dumps(obj).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body = PAGE.encode()
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith("/action"):
                    cmd = ""
                    if "cmd=" in self.path:
                        cmd = self.path.split("cmd=", 1)[1].split("&", 1)[0]
                    try:
                        view._on_action(cmd)
                    except Exception as e:
                        with view._lock:
                            view._status["msg"] = f"action error: {e}"
                    with view._lock:
                        self._json(dict(view._status))
                    return
                if self.path == "/state":
                    with view._lock:
                        self._json(dict(view._status))
                    return
                if self.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type",
                                     "multipart/x-mixed-replace; boundary=FRAME")
                    self.send_header("Cache-Control", "no-cache, private")
                    self.end_headers()
                    last = None
                    try:
                        while not view._quit:
                            with view._lock:
                                buf = view._frame_jpeg
                            if buf and buf is not last:
                                self.wfile.write(b"--FRAME\r\n")
                                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                                self.wfile.write(f"Content-Length: {len(buf)}\r\n\r\n".encode())
                                self.wfile.write(buf)
                                self.wfile.write(b"\r\n")
                                last = buf
                            time.sleep(0.05)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                self.send_response(404)
                self.end_headers()

        return Handler
