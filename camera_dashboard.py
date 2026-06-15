#!/usr/bin/env python3
"""USB Camera Dashboard — live feed, settings, sensor info."""

import cv2
import threading
import time
import json
import os
import glob as _glob
from datetime import datetime
from flask import Flask, Response, jsonify, request, render_template_string

RECORDINGS_DIR = os.path.expanduser("~/camera_recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)

# Pull real device names from AVFoundation if available
def _av_device_names():
    try:
        import AVFoundation as av
        devices = av.AVCaptureDevice.devicesWithMediaType_(av.AVMediaTypeVideo)
        return {i: d.localizedName() for i, d in enumerate(devices)}
    except Exception:
        return {}

app = Flask(__name__)

# ── Camera state ─────────────────────────────────────────────────────────────

class CameraManager:
    def __init__(self):
        self.cap = None
        self.index = -1
        self.name = ""
        self.lock = threading.Lock()
        self.frame = None
        self.running = False
        self._thread = None
        self.writable_props = set()   # props confirmed writable by this camera

    # ── Camera enumeration ──────────────────────────────────────────────────

    def list_cameras(self):
        # Use AVFoundation directly — instant, no frame-read needed
        try:
            import AVFoundation as av
            devices = av.AVCaptureDevice.devicesWithMediaType_(av.AVMediaTypeVideo)
            return [
                {"index": i, "name": d.localizedName(), "backend": "AVFOUNDATION"}
                for i, d in enumerate(devices)
            ]
        except Exception:
            pass
        # Fallback: OpenCV scan with warmup
        cameras = []
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if not cap.isOpened():
                cap.release()
                continue
            ok = False
            for _ in range(20):
                ok, _ = cap.read()
                if ok:
                    break
                time.sleep(0.05)
            if ok:
                cameras.append({"index": i, "name": f"Camera {i}", "backend": "unknown"})
            cap.release()
        return cameras

    # ── Open / close ────────────────────────────────────────────────────────

    def open(self, index):
        # Stop existing capture first
        with self.lock:
            self.running = False
            old_cap = self.cap
            self.cap = None
        if old_cap:
            old_cap.release()
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None

        # cv2.VideoCapture() itself can block indefinitely on macOS for
        # problematic cameras — run it in a thread with a hard timeout.
        PROBE_VALUES = {
            "brightness": 0.5, "contrast": 0.5, "saturation": 0.5,
            "hue": 10.0, "gain": 10.0, "exposure": -6.0,
            "sharpness": 50.0, "gamma": 100.0, "backlight": 1.0,
            "zoom": 110.0, "focus": 128.0, "white_balance_u": 4000.0,
            "white_balance_v": 4000.0, "iso_speed": 100.0,
            "pan": 1.0, "tilt": 1.0, "roll": 1.0, "iris": 1.0,
            "temperature": 4000.0, "autofocus": 1.0,
            "auto_exposure": 1.0, "auto_wb": 1.0, "monochrome": 1.0,
            "fps": 30.0, "width": 640.0, "height": 480.0,
        }
        result = {"cap": None, "frame": None, "writable": set()}

        def _open():
            cap = cv2.VideoCapture(index)
            if not cap.isOpened():
                return
            # Warmup
            for _ in range(40):
                ok, frame = cap.read()
                if ok:
                    result["cap"] = cap
                    result["frame"] = frame
                    break
                time.sleep(0.05)
            if result["cap"] is None:
                cap.release()
                return
            # Probe writable props — also inside the timeout thread
            writable = set()
            for pname, pid in self.PROPS.items():
                if pname in ("fourcc", "trigger", "trigger_delay"):
                    continue
                cur = cap.get(pid)
                probe = PROBE_VALUES.get(pname, 1.0)
                test_val = probe if abs(cur - probe) > 0.001 else probe + 1.0
                if cap.set(pid, test_val):
                    writable.add(pname)
                    cap.set(pid, cur)
            result["writable"] = writable

        t = threading.Thread(target=_open, daemon=True)
        t.start()
        t.join(timeout=10)  # generous timeout covers warmup + probe

        cap = result["cap"]
        if cap is None:
            return False

        av_names = _av_device_names()

        with self.lock:
            self.cap = cap
            self.frame = result["frame"]
            self.index = index
            self.name = av_names.get(index, f"Camera {index}")
            self.running = True
            self.writable_props = result["writable"]

        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        return True

    def close(self):
        self.running = False
        with self.lock:
            if self.cap:
                self.cap.release()
                self.cap = None

    # ── Capture loop ────────────────────────────────────────────────────────

    def _capture_loop(self):
        while self.running:
            with self.lock:
                if self.cap and self.cap.isOpened():
                    ok, frame = self.cap.read()
                    if ok:
                        self.frame = frame
                        recorder.write(frame)
            time.sleep(0.01)  # ~100 fps max

    # ── MJPEG frame generator ───────────────────────────────────────────────

    def gen_frames(self):
        while True:
            # Serve playback frame if player is active, else live camera frame
            frame = player.frame if player.running else self.frame
            if frame is None:
                time.sleep(0.05)
                continue
            ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                continue
            yield (
                b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                + buf.tobytes()
                + b"\r\n"
            )
            time.sleep(0.033)  # ~30 fps

    # ── Property helpers ────────────────────────────────────────────────────

    # Map of OpenCV property IDs we expose
    PROPS = {
        "brightness":        cv2.CAP_PROP_BRIGHTNESS,
        "contrast":          cv2.CAP_PROP_CONTRAST,
        "saturation":        cv2.CAP_PROP_SATURATION,
        "hue":               cv2.CAP_PROP_HUE,
        "gain":              cv2.CAP_PROP_GAIN,
        "exposure":          cv2.CAP_PROP_EXPOSURE,
        "white_balance_u":   cv2.CAP_PROP_WHITE_BALANCE_BLUE_U,
        "white_balance_v":   cv2.CAP_PROP_WHITE_BALANCE_RED_V,
        "sharpness":         cv2.CAP_PROP_SHARPNESS,
        "gamma":             cv2.CAP_PROP_GAMMA,
        "backlight":         cv2.CAP_PROP_BACKLIGHT,
        "zoom":              cv2.CAP_PROP_ZOOM,
        "focus":             cv2.CAP_PROP_FOCUS,
        "autofocus":         cv2.CAP_PROP_AUTOFOCUS,
        "auto_exposure":     cv2.CAP_PROP_AUTO_EXPOSURE,
        "auto_wb":           cv2.CAP_PROP_AUTO_WB,
        "fps":               cv2.CAP_PROP_FPS,
        "width":             cv2.CAP_PROP_FRAME_WIDTH,
        "height":            cv2.CAP_PROP_FRAME_HEIGHT,
        "fourcc":            cv2.CAP_PROP_FOURCC,
        "iso_speed":         cv2.CAP_PROP_ISO_SPEED,
        "pan":               cv2.CAP_PROP_PAN,
        "tilt":              cv2.CAP_PROP_TILT,
        "roll":              cv2.CAP_PROP_ROLL,
        "iris":              cv2.CAP_PROP_IRIS,
        "temperature":       cv2.CAP_PROP_TEMPERATURE,
        "trigger":           cv2.CAP_PROP_TRIGGER,
        "trigger_delay":     cv2.CAP_PROP_TRIGGER_DELAY,
        "monochrome":        cv2.CAP_PROP_MONOCHROME,
    }

    def get_all_props(self):
        result = {}
        with self.lock:
            if not self.cap or not self.cap.isOpened():
                return result
            writable = self.writable_props
            for name, pid in self.PROPS.items():
                val = self.cap.get(pid)
                result[name] = {"value": val, "writable": name in writable}
        # Decode FourCC
        if "fourcc" in result:
            raw = int(result["fourcc"]["value"])
            try:
                cc = "".join([chr((raw >> (8 * i)) & 0xFF) for i in range(4)]).strip("\x00")
                result["fourcc"]["fourcc_str"] = cc if cc.isprintable() else str(raw)
            except Exception:
                result["fourcc"]["fourcc_str"] = str(raw)
        return result

    def set_prop(self, name, value):
        pid = self.PROPS.get(name)
        if pid is None:
            return False, "unknown property"
        with self.lock:
            if not self.cap or not self.cap.isOpened():
                return False, "no camera open"
            ok = self.cap.set(pid, float(value))
        return ok, "ok" if ok else "camera rejected value"

    def get_sensor_info(self):
        """Return sensor / format capabilities."""
        info = {}
        with self.lock:
            if not self.cap or not self.cap.isOpened():
                return info
            try:
                info["backend"] = self.cap.getBackendName()
            except Exception:
                info["backend"] = "unknown"
            info["frame_width"]  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            info["frame_height"] = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            info["fps"]          = self.cap.get(cv2.CAP_PROP_FPS)
            info["format"]       = int(self.cap.get(cv2.CAP_PROP_FORMAT))
            info["mode"]         = int(self.cap.get(cv2.CAP_PROP_MODE))
            raw = int(self.cap.get(cv2.CAP_PROP_FOURCC))
            try:
                cc = "".join([chr((raw >> (8 * i)) & 0xFF) for i in range(4)]).strip("\x00")
                info["codec"] = cc if cc.isprintable() else str(raw)
            except Exception:
                info["codec"] = str(raw)
            info["buffer_size"]  = int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE))
            info["channel"]      = int(self.cap.get(cv2.CAP_PROP_CHANNEL))
            info["convert_rgb"]  = int(self.cap.get(cv2.CAP_PROP_CONVERT_RGB))
        return info


cam = CameraManager()

# ── Recorder ──────────────────────────────────────────────────────────────────

class Recorder:
    def __init__(self):
        self.writer = None
        self.lock = threading.Lock()
        self.filename = ""
        self.start_time = None
        self.frame_count = 0

    def start(self, width, height, fps):
        with self.lock:
            if self.writer:
                return False, "already recording"
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            fps = fps if fps > 0 else 30.0
            path = os.path.join(RECORDINGS_DIR, f"rec_{ts}.avi")
            # MJPG gives near-lossless quality in a standard container
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.writer = cv2.VideoWriter(path, fourcc, fps, (width, height))
            if not self.writer.isOpened():
                self.writer = None
                return False, "VideoWriter failed to open"
            self.filename = path
            self.start_time = time.time()
            self.frame_count = 0
            return True, path

    def write(self, frame):
        with self.lock:
            if self.writer and self.writer.isOpened():
                self.writer.write(frame)
                self.frame_count += 1

    def stop(self):
        with self.lock:
            if not self.writer:
                return False, "not recording"
            self.writer.release()
            self.writer = None
            path = self.filename
            elapsed = time.time() - self.start_time
            frames = self.frame_count
            self.filename = ""
            self.start_time = None
            self.frame_count = 0
            return True, {"path": path, "duration": round(elapsed, 2), "frames": frames}

    def status(self):
        with self.lock:
            if not self.writer:
                return {"recording": False}
            return {
                "recording": True,
                "filename": os.path.basename(self.filename),
                "elapsed": round(time.time() - self.start_time, 1),
                "frames": self.frame_count,
            }

recorder = Recorder()

# ── Playback ──────────────────────────────────────────────────────────────────

class Player:
    def __init__(self):
        self.cap = None
        self.lock = threading.Lock()
        self.frame = None
        self.running = False
        self.filename = ""
        self.total_frames = 0
        self.current_frame = 0
        self._thread = None

    def start(self, path):
        self.stop()
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            return False, "could not open file"
        with self.lock:
            self.cap = cap
            self.filename = path
            self.total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
            self.current_frame = 0
            self.running = True
        self._thread = threading.Thread(target=self._play_loop, daemon=True)
        self._thread.start()
        return True, "ok"

    def _play_loop(self):
        fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        delay = 1.0 / fps
        while self.running:
            with self.lock:
                if not self.cap:
                    break
                ok, frame = self.cap.read()
                if ok:
                    self.frame = frame
                    self.current_frame += 1
                else:
                    # Loop back to start
                    self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    self.current_frame = 0
            time.sleep(delay)

    def stop(self):
        self.running = False
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        with self.lock:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.frame = None
            self.filename = ""

    def status(self):
        with self.lock:
            if not self.cap:
                return {"playing": False}
            pct = (self.current_frame / self.total_frames * 100) if self.total_frames else 0
            return {
                "playing": True,
                "filename": os.path.basename(self.filename),
                "current_frame": self.current_frame,
                "total_frames": self.total_frames,
                "progress_pct": round(pct, 1),
            }

player = Player()

# ── Routes ─────────────────────────────────────────────────────────────────────

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>USB Camera Dashboard</title>
<style>
  :root {
    --bg: #0f1117; --panel: #1a1d27; --border: #2e3148;
    --accent: #5c6ef8; --text: #e4e6f0; --muted: #6b7280;
    --green: #22c55e; --red: #ef4444; --yellow: #f59e0b;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text); font-family: 'SF Mono', 'Menlo', monospace; font-size: 13px; }
  header { background: var(--panel); border-bottom: 1px solid var(--border); padding: 12px 20px; display: flex; align-items: center; gap: 16px; }
  header h1 { font-size: 15px; font-weight: 700; letter-spacing: .04em; color: var(--accent); }
  .badge { padding: 2px 8px; border-radius: 4px; font-size: 11px; font-weight: 600; }
  .badge-green { background: rgba(34,197,94,.15); color: var(--green); border: 1px solid rgba(34,197,94,.3); }
  .badge-red { background: rgba(239,68,68,.15); color: var(--red); border: 1px solid rgba(239,68,68,.3); }
  #status-dot { width: 8px; height: 8px; border-radius: 50%; background: var(--muted); flex-shrink: 0; }
  #status-dot.live { background: var(--green); box-shadow: 0 0 6px var(--green); }

  .layout { display: grid; grid-template-columns: 1fr 340px; height: calc(100vh - 49px); overflow: hidden; }

  /* Feed pane */
  .feed-pane { padding: 16px; display: flex; flex-direction: column; gap: 12px; overflow-y: auto; }
  .cam-bar { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  select, button, input[type=number] { background: var(--panel); border: 1px solid var(--border); color: var(--text); border-radius: 6px; padding: 6px 10px; font-family: inherit; font-size: 12px; cursor: pointer; }
  select:focus, input:focus { outline: 2px solid var(--accent); }
  button { font-weight: 600; transition: background .15s; }
  button:hover { background: var(--border); }
  button.primary { background: var(--accent); border-color: var(--accent); color: #fff; }
  button.primary:hover { background: #4a5ce6; }
  button.danger { background: rgba(239,68,68,.15); border-color: rgba(239,68,68,.4); color: var(--red); }

  .video-box { position: relative; background: #000; border-radius: 8px; border: 1px solid var(--border); overflow: hidden; display: flex; align-items: center; justify-content: center; min-height: 300px; }
  #feed { max-width: 100%; max-height: 60vh; display: block; }
  #no-feed { color: var(--muted); font-size: 14px; padding: 40px; }

  /* Sensor info */
  .info-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 8px; }
  .info-card { background: var(--panel); border: 1px solid var(--border); border-radius: 8px; padding: 10px 12px; }
  .info-card .label { color: var(--muted); font-size: 11px; text-transform: uppercase; letter-spacing: .05em; }
  .info-card .val { font-size: 14px; font-weight: 700; margin-top: 2px; color: var(--accent); }

  /* Right panel */
  .settings-pane { border-left: 1px solid var(--border); background: var(--panel); display: flex; flex-direction: column; overflow: hidden; }
  .pane-header { padding: 12px 16px; font-weight: 700; font-size: 12px; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); border-bottom: 1px solid var(--border); flex-shrink: 0; }
  .settings-scroll { flex: 1; overflow-y: auto; padding: 12px 16px; display: flex; flex-direction: column; gap: 14px; }

  .setting-row { display: flex; flex-direction: column; gap: 4px; }
  .setting-row .row-top { display: flex; justify-content: space-between; align-items: center; }
  .setting-row label { font-size: 11px; color: var(--muted); text-transform: uppercase; letter-spacing: .04em; }
  .setting-row .cur-val { font-size: 11px; font-weight: 700; color: var(--text); min-width: 52px; text-align: right; }
  input[type=range] { width: 100%; accent-color: var(--accent); cursor: pointer; }
  .manual-input { display: flex; gap: 6px; margin-top: 2px; }
  .manual-input input[type=number] { flex: 1; padding: 4px 6px; }
  .manual-input button { padding: 4px 8px; font-size: 11px; }

  .section-title { font-size: 11px; font-weight: 700; text-transform: uppercase; letter-spacing: .06em; color: var(--muted); padding-bottom: 4px; border-bottom: 1px solid var(--border); }
  ::-webkit-scrollbar { width: 6px; } ::-webkit-scrollbar-track { background: transparent; } ::-webkit-scrollbar-thumb { background: var(--border); border-radius: 3px; }
  @keyframes blink { 0%,100%{opacity:1} 50%{opacity:0} }
  .toast { position: fixed; bottom: 20px; right: 20px; background: var(--panel); border: 1px solid var(--border); border-radius: 8px; padding: 10px 16px; font-size: 12px; opacity: 0; transition: opacity .3s; pointer-events: none; z-index: 99; }
  .toast.show { opacity: 1; }
  .toast.ok { border-color: rgba(34,197,94,.5); color: var(--green); }
  .toast.err { border-color: rgba(239,68,68,.5); color: var(--red); }
</style>
</head>
<body>
<header>
  <div id="status-dot"></div>
  <h1>USB CAMERA DASHBOARD</h1>
  <span id="cam-badge" class="badge badge-red">NO CAMERA</span>
</header>

<div class="layout">
  <!-- Left: feed + sensor info -->
  <div class="feed-pane">
    <div class="cam-bar">
      <select id="cam-select"><option value="">-- select camera --</option></select>
      <button class="primary" onclick="openCam()">Connect</button>
      <button class="danger" onclick="closeCam()">Disconnect</button>
      <button onclick="refreshCams()">&#8635; Refresh</button>
      <button onclick="loadProps()">&#8635; Reload Settings</button>
    </div>

    <div class="video-box">
      <img id="feed" src="" alt="" style="display:none">
      <div id="no-feed">No camera connected</div>
      <div id="rec-indicator" style="display:none;position:absolute;top:10px;left:10px;background:rgba(0,0,0,.6);border-radius:6px;padding:4px 10px;display:none;align-items:center;gap:6px;font-size:12px;font-weight:700;">
        <div style="width:8px;height:8px;border-radius:50%;background:var(--red);animation:blink 1s infinite"></div>
        <span id="rec-timer">REC 0:00</span>
      </div>
      <div id="play-indicator" style="display:none;position:absolute;top:10px;left:10px;background:rgba(0,0,0,.6);border-radius:6px;padding:4px 10px;font-size:12px;font-weight:700;color:var(--green)">
        ▶ <span id="play-progress">0%</span>
      </div>
    </div>

    <!-- Record controls -->
    <div class="cam-bar" style="border-top:1px solid var(--border);padding-top:10px">
      <span style="color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.05em">Record</span>
      <button id="btn-record" style="background:rgba(239,68,68,.15);border-color:rgba(239,68,68,.4);color:var(--red);font-weight:700" onclick="startRecording()">⏺ Record</button>
      <button id="btn-stop-rec" style="display:none" onclick="stopRecording()">⏹ Stop</button>
      <span style="color:var(--muted);font-size:11px;text-transform:uppercase;letter-spacing:.05em;margin-left:8px">Playback</span>
      <select id="rec-select"><option value="">-- select recording --</option></select>
      <button onclick="startPlayback()">▶ Play</button>
      <button onclick="stopPlayback()">⏹ Stop</button>
      <button onclick="loadRecordings()">&#8635;</button>
    </div>

    <div class="section-title" style="padding-top:4px">Sensor &amp; Format Info</div>
    <div id="info-grid" class="info-grid"></div>
  </div>

  <!-- Right: settings -->
  <div class="settings-pane">
    <div class="pane-header">Camera Settings</div>
    <div class="settings-scroll" id="settings-container">
      <div style="color:var(--muted);font-size:12px;margin-top:40px;text-align:center">Connect a camera to see settings</div>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
// ── Config ─────────────────────────────────────────────────────────────────

const RANGE_HINTS = {
  brightness:      [-1, 1],
  contrast:        [-1, 1],
  saturation:      [-1, 1],
  hue:             [-180, 180],
  gain:            [0, 100],
  exposure:        [-13, 0],
  sharpness:       [0, 100],
  gamma:           [0, 500],
  backlight:       [0, 3],
  zoom:            [100, 400],
  focus:           [0, 255],
  white_balance_u: [2000, 6500],
  white_balance_v: [2000, 6500],
  iso_speed:       [0, 3200],
  pan:             [-180, 180],
  tilt:            [-180, 180],
  roll:            [-180, 180],
  iris:            [0, 100],
  temperature:     [2800, 6500],
  fps:             [1, 120],
  width:           [160, 3840],
  height:          [120, 2160],
};

const BOOL_PROPS = new Set(['autofocus', 'auto_exposure', 'auto_wb', 'monochrome', 'trigger']);
const READ_ONLY  = new Set(['fourcc', 'fourcc_str', 'buffer_size', 'channel', 'convert_rgb', 'format', 'mode', 'backend']);
const SKIP_PROPS = new Set(['fourcc_str']); // shown inside fourcc card

// ── State ──────────────────────────────────────────────────────────────────

let liveIndex = null;
let pollTimer = null;

// ── Camera list ────────────────────────────────────────────────────────────

let lastCamJson = '';
async function refreshCams(silent=false) {
  try {
    const res = await fetch('/cameras');
    const cams = await res.json();
    const json = JSON.stringify(cams);
    const changed = json !== lastCamJson;
    lastCamJson = json;
    if (!changed && silent) return;  // no change, skip rebuild
    const sel = document.getElementById('cam-select');
    const prev = sel.value;
    sel.innerHTML = '<option value="">-- select camera --</option>';
    cams.forEach(c => {
      const o = document.createElement('option');
      o.value = c.index;
      o.textContent = `${c.name}  [${c.backend}]`;
      sel.appendChild(o);
    });
    if (prev) sel.value = prev;
    if (changed && silent) showToast(`${cams.length} camera(s) detected`, 'ok');
  } catch(e) {}
}

// ── Open / close ───────────────────────────────────────────────────────────

async function openCam() {
  const idx = document.getElementById('cam-select').value;
  if (idx === '') { showToast('Select a camera first', 'err'); return; }
  const res = await fetch('/open', { method: 'POST', headers: {'Content-Type':'application/json'}, body: JSON.stringify({index: parseInt(idx)}) });
  const data = await res.json();
  if (data.ok) {
    liveIndex = parseInt(idx);
    const feed = document.getElementById('feed');
    feed.src = '/feed?' + Date.now();
    feed.style.display = 'block';
    document.getElementById('no-feed').style.display = 'none';
    document.getElementById('status-dot').classList.add('live');
    document.getElementById('cam-badge').className = 'badge badge-green';
    // Fetch real device name from server
    const nameRes = await fetch('/camera_name');
    const nameData = await nameRes.json();
    document.getElementById('cam-badge').textContent = nameData.name || `CAM ${idx}`;
    loadProps();
    loadSensorInfo();
    startPoll();
    showToast('Camera connected', 'ok');
  } else {
    showToast('Failed to open camera', 'err');
  }
}

async function closeCam() {
  clearInterval(pollTimer);
  await fetch('/close', { method: 'POST' });
  liveIndex = null;
  document.getElementById('feed').style.display = 'none';
  document.getElementById('feed').src = '';
  document.getElementById('no-feed').style.display = '';
  document.getElementById('status-dot').classList.remove('live');
  document.getElementById('cam-badge').textContent = 'NO CAMERA';
  document.getElementById('cam-badge').className = 'badge badge-red';
  document.getElementById('settings-container').innerHTML = '<div style="color:var(--muted);font-size:12px;margin-top:40px;text-align:center">Connect a camera to see settings</div>';
  document.getElementById('info-grid').innerHTML = '';
  showToast('Disconnected', 'ok');
}

// ── Sensor info ────────────────────────────────────────────────────────────

async function loadSensorInfo() {
  const res = await fetch('/sensor_info');
  const info = await res.json();
  const grid = document.getElementById('info-grid');
  grid.innerHTML = '';
  const order = ['backend','frame_width','frame_height','fps','codec','format','mode','buffer_size','channel','convert_rgb'];
  const display = (k) => k.replace(/_/g,' ').toUpperCase();
  const fmt = (k, v) => {
    if (k === 'fps') return parseFloat(v).toFixed(1);
    if (typeof v === 'number' && !Number.isInteger(v)) return parseFloat(v).toFixed(2);
    return v;
  };
  const keys = [...new Set([...order, ...Object.keys(info)])];
  keys.forEach(k => {
    if (!(k in info)) return;
    const card = document.createElement('div');
    card.className = 'info-card';
    card.innerHTML = `<div class="label">${display(k)}</div><div class="val">${fmt(k, info[k])}</div>`;
    grid.appendChild(card);
  });
}

// ── Settings ───────────────────────────────────────────────────────────────

async function loadProps() {
  const res = await fetch('/props');
  const props = await res.json();
  buildSettingsUI(props);
}

function buildSettingsUI(props) {
  const container = document.getElementById('settings-container');
  container.innerHTML = '';

  const WRITABLE = [], RO = [];
  Object.entries(props).forEach(([k, entry]) => {
    if (SKIP_PROPS.has(k)) return;
    if (entry.writable) WRITABLE.push([k, entry]);
    else RO.push([k, entry]);
  });

  if (WRITABLE.length === 0 && RO.length === 0) {
    container.innerHTML = '<div style="color:var(--muted);font-size:12px;margin-top:40px;text-align:center">No properties available</div>';
    return;
  }

  if (WRITABLE.length) {
    const t = document.createElement('div');
    t.className = 'section-title';
    t.textContent = `Adjustable (${WRITABLE.length})`;
    container.appendChild(t);
    WRITABLE.forEach(([k, entry]) => container.appendChild(buildRow(k, entry.value)));
  }

  if (RO.length) {
    const t = document.createElement('div');
    t.className = 'section-title';
    t.style.marginTop = '12px';
    t.textContent = `Read-only (${RO.length})`;
    container.appendChild(t);
    RO.forEach(([k, entry]) => {
      const d = document.createElement('div');
      d.className = 'setting-row';
      let display = entry.value;
      if (k === 'fourcc' && entry.fourcc_str) display = `${entry.fourcc_str}`;
      else if (typeof display === 'number') display = parseFloat(display).toFixed(2);
      d.innerHTML = `<div class="row-top"><label>${k.replace(/_/g,' ')}</label><span class="cur-val">${display}</span></div>`;
      container.appendChild(d);
    });
  }
}

function buildRow(key, val) {
  const row = document.createElement('div');
  row.className = 'setting-row';
  row.id = `row-${key}`;

  if (BOOL_PROPS.has(key)) {
    row.innerHTML = `
      <div class="row-top">
        <label>${key.replace(/_/g,' ')}</label>
        <span class="cur-val" id="val-${key}">${val}</span>
      </div>
      <div style="display:flex;gap:6px;margin-top:4px">
        <button style="flex:1;font-size:11px" onclick="setProp('${key}', 1)">ON</button>
        <button style="flex:1;font-size:11px" onclick="setProp('${key}', 0)">OFF</button>
      </div>`;
    return row;
  }

  const hint = RANGE_HINTS[key] || [0, 100];
  const [mn, mx] = hint;
  const step = (mx - mn) <= 2 ? 0.01 : 1;

  row.innerHTML = `
    <div class="row-top">
      <label>${key.replace(/_/g,' ')}</label>
      <span class="cur-val" id="val-${key}">${typeof val === 'number' ? val.toFixed(2) : val}</span>
    </div>
    <input type="range" id="slider-${key}" min="${mn}" max="${mx}" step="${step}" value="${val}"
      oninput="document.getElementById('val-${key}').textContent=parseFloat(this.value).toFixed(2);document.getElementById('num-${key}').value=this.value"
      onchange="setProp('${key}', parseFloat(this.value))">
    <div class="manual-input">
      <input type="number" id="num-${key}" value="${val}" min="${mn}" max="${mx}" step="${step}">
      <button onclick="setPropFromNum('${key}')">Set</button>
    </div>`;
  return row;
}

async function setProp(key, value) {
  const res = await fetch('/set_prop', {
    method: 'POST',
    headers: {'Content-Type':'application/json'},
    body: JSON.stringify({name: key, value: value})
  });
  const data = await res.json();
  if (data.ok) {
    updateDisplay(key, value);
    showToast(`${key} = ${value}`, 'ok');
  } else {
    showToast(`${key}: ${data.message}`, 'err');
  }
}

function setPropFromNum(key) {
  const num = document.getElementById(`num-${key}`);
  if (!num) return;
  setProp(key, parseFloat(num.value));
}

function updateDisplay(key, val) {
  const el = document.getElementById(`val-${key}`);
  if (el) el.textContent = typeof val === 'number' ? parseFloat(val).toFixed(2) : val;
  const sl = document.getElementById(`slider-${key}`);
  if (sl) sl.value = val;
  const nm = document.getElementById(`num-${key}`);
  if (nm) nm.value = val;
}

// ── Live prop polling ──────────────────────────────────────────────────────

function startPoll() {
  clearInterval(pollTimer);
  pollTimer = setInterval(async () => {
    if (liveIndex === null) return;
    const res = await fetch('/props');
    const props = await res.json();
    Object.entries(props).forEach(([k, entry]) => {
      const el = document.getElementById(`val-${k}`);
      if (el && !isSliderBeingDragged(k)) {
        const v = entry.value;
        el.textContent = typeof v === 'number' ? parseFloat(v).toFixed(2) : v;
      }
    });
    loadSensorInfo();
  }, 3000);
}

const dragging = new Set();
document.addEventListener('mousedown', e => { if (e.target.type === 'range') dragging.add(e.target.id.replace('slider-','')) });
document.addEventListener('mouseup', () => dragging.clear());
function isSliderBeingDragged(k) { return dragging.has(k); }

// ── Toast ──────────────────────────────────────────────────────────────────

let toastTimer;
function showToast(msg, type='ok') {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = `toast ${type} show`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 2500);
}

// ── Recording ──────────────────────────────────────────────────────────────

let recPollTimer = null;

async function startRecording() {
  const res = await fetch('/record/start', {method:'POST'});
  const data = await res.json();
  if (data.ok) {
    document.getElementById('btn-record').style.display = 'none';
    document.getElementById('btn-stop-rec').style.display = '';
    document.getElementById('rec-indicator').style.display = 'flex';
    showToast(`Recording: ${data.message}`, 'ok');
    recPollTimer = setInterval(updateRecTimer, 500);
  } else {
    showToast(data.message, 'err');
  }
}

async function updateRecTimer() {
  const res = await fetch('/record/status');
  const s = await res.json();
  if (s.recording) {
    const m = Math.floor(s.elapsed/60), sec = Math.floor(s.elapsed%60);
    document.getElementById('rec-timer').textContent = `REC ${m}:${String(sec).padStart(2,'0')}`;
  }
}

async function stopRecording() {
  clearInterval(recPollTimer);
  const res = await fetch('/record/stop', {method:'POST'});
  const data = await res.json();
  document.getElementById('btn-record').style.display = '';
  document.getElementById('btn-stop-rec').style.display = 'none';
  document.getElementById('rec-indicator').style.display = 'none';
  if (data.ok) {
    const r = data.result;
    showToast(`Saved ${r.duration}s  ${r.frames} frames`, 'ok');
    loadRecordings();
  }
}

// ── Playback ───────────────────────────────────────────────────────────────

let playPollTimer = null;

async function loadRecordings() {
  const res = await fetch('/recordings');
  const recs = await res.json();
  const sel = document.getElementById('rec-select');
  sel.innerHTML = '<option value="">-- select recording --</option>';
  recs.forEach(r => {
    const o = document.createElement('option');
    o.value = r.filename;
    o.textContent = `${r.filename}  (${r.duration}s, ${r.size_mb}MB)`;
    sel.appendChild(o);
  });
}

async function startPlayback() {
  const filename = document.getElementById('rec-select').value;
  if (!filename) { showToast('Select a recording first', 'err'); return; }
  await fetch('/playback/stop', {method:'POST'});
  const res = await fetch('/playback/start', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({filename})
  });
  const data = await res.json();
  if (data.ok) {
    // Switch feed to playback stream
    const feed = document.getElementById('feed');
    feed.src = '/feed?' + Date.now();
    feed.style.display = 'block';
    document.getElementById('no-feed').style.display = 'none';
    document.getElementById('play-indicator').style.display = 'block';
    showToast(`Playing ${filename}`, 'ok');
    playPollTimer = setInterval(updatePlayProgress, 500);
  } else {
    showToast(data.message, 'err');
  }
}

async function updatePlayProgress() {
  const res = await fetch('/playback/status');
  const s = await res.json();
  if (s.playing) {
    document.getElementById('play-progress').textContent = `${s.progress_pct}%  ${s.current_frame}/${s.total_frames}`;
  } else {
    clearInterval(playPollTimer);
    document.getElementById('play-indicator').style.display = 'none';
  }
}

async function stopPlayback() {
  clearInterval(playPollTimer);
  await fetch('/playback/stop', {method:'POST'});
  document.getElementById('play-indicator').style.display = 'none';
  showToast('Playback stopped', 'ok');
}

// ── Init ───────────────────────────────────────────────────────────────────

refreshCams();
loadRecordings();
// Auto-detect newly plugged cameras every 2 seconds
setInterval(() => refreshCams(true), 2000);
</script>
</body>
</html>
"""

@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/cameras")
def cameras():
    return jsonify(cam.list_cameras())

@app.route("/open", methods=["POST"])
def open_cam():
    data = request.json
    ok = cam.open(data["index"])
    return jsonify({"ok": ok})

@app.route("/close", methods=["POST"])
def close_cam():
    cam.close()
    return jsonify({"ok": True})

@app.route("/feed")
def feed():
    return Response(cam.gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/props")
def props():
    return jsonify(cam.get_all_props())

@app.route("/set_prop", methods=["POST"])
def set_prop():
    data = request.json
    ok, msg = cam.set_prop(data["name"], data["value"])
    return jsonify({"ok": ok, "message": msg})

@app.route("/sensor_info")
def sensor_info():
    return jsonify(cam.get_sensor_info())

@app.route("/camera_name")
def camera_name():
    return jsonify({"name": cam.name, "index": cam.index})

# ── Recording routes ──────────────────────────────────────────────────────────

@app.route("/record/start", methods=["POST"])
def record_start():
    with cam.lock:
        if not cam.cap or not cam.cap.isOpened():
            return jsonify({"ok": False, "message": "No camera open"})
        w = int(cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cam.cap.get(cv2.CAP_PROP_FPS) or 30.0
    ok, result = recorder.start(w, h, fps)
    return jsonify({"ok": ok, "message": result if not ok else os.path.basename(result)})

@app.route("/record/stop", methods=["POST"])
def record_stop():
    ok, result = recorder.stop()
    return jsonify({"ok": ok, "result": result})

@app.route("/record/status")
def record_status():
    return jsonify(recorder.status())

@app.route("/recordings")
def list_recordings():
    files = sorted(_glob.glob(os.path.join(RECORDINGS_DIR, "*.avi")), reverse=True)
    out = []
    for f in files:
        stat = os.stat(f)
        cap = cv2.VideoCapture(f)
        duration = 0
        if cap.isOpened():
            fc = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            fps = cap.get(cv2.CAP_PROP_FPS) or 30
            duration = round(fc / fps, 1)
            cap.release()
        out.append({
            "filename": os.path.basename(f),
            "size_mb": round(stat.st_size / 1e6, 1),
            "duration": duration,
            "modified": datetime.fromtimestamp(stat.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
        })
    return jsonify(out)

# ── Playback routes ───────────────────────────────────────────────────────────

@app.route("/playback/start", methods=["POST"])
def playback_start():
    filename = request.json.get("filename")
    path = os.path.join(RECORDINGS_DIR, filename)
    if not os.path.exists(path):
        return jsonify({"ok": False, "message": "File not found"})
    ok, msg = player.start(path)
    return jsonify({"ok": ok, "message": msg})

@app.route("/playback/stop", methods=["POST"])
def playback_stop():
    player.stop()
    return jsonify({"ok": True})

@app.route("/playback/status")
def playback_status():
    return jsonify(player.status())


if __name__ == "__main__":
    print("\n  USB Camera Dashboard")
    print("  Open  →  http://localhost:5050\n")
    app.run(host="0.0.0.0", port=5050, debug=False, threaded=True)
