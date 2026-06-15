# USB Camera Dashboard

A local web app to connect to any USB camera, view the live feed, adjust settings, and record/play back footage.

## Features
- Auto-detects all connected USB cameras (hot-plug detection)
- Live MJPEG feed in the browser
- Shows only the settings the camera actually supports (brightness, contrast, exposure, etc.)
- Record to AVI (MJPEG, near-lossless)
- Playback recordings through the same feed window
- Real camera names via AVFoundation (macOS)

## Requirements

```bash
pip3 install flask opencv-python pyobjc-framework-AVFoundation
```

> `pyobjc-framework-AVFoundation` is macOS only — on Linux/Windows it will fall back to generic camera names.

## Run

```bash
python3 camera_dashboard.py
```

Then open **http://localhost:5050** in your browser.

## Usage

1. Select a camera from the dropdown and click **Connect**
2. Adjust settings in the right panel (only supported controls shown)
3. Click **⏺ Record** to start recording — saved to `~/camera_recordings/`
4. Use the **Playback** dropdown to replay any recording
