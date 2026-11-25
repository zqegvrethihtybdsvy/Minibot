#!/usr/bin/env python3
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import cv2
from picamera2 import Picamera2

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SAVE_DIR = os.path.join(SCRIPT_DIR, "photos")
os.makedirs(SAVE_DIR, exist_ok=True)

latest_jpeg = None
latest_frame = None
lock = threading.Lock()
save_lock = threading.Lock()
photo_count = 1
running = True

# 1. Fast Camera Setup (2 buffers = minimal latency)
picam2 = Picamera2()

config = picam2.create_preview_configuration(
    main={"size": (1640, 1232), "format": "RGB888"},
    buffer_count=2,
    controls={"FrameRate": 120},
)
picam2.configure(config)
picam2.start()
time.sleep(0.2)  # Short auto-exposure settle


def capture_loop():
    global latest_jpeg, latest_frame
    while running:
        frame = picam2.capture_array()
        # Encode JPEG for browser live preview (quality 75 = fast over Wi-Fi)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with lock:
            latest_frame = frame
            if ok:
                latest_jpeg = buf.tobytes()


def save_snapshot():
    global photo_count
    with lock:
        snap = None if latest_frame is None else latest_frame.copy()
    if snap is None:
        return None
    with save_lock:
        filename = f"calib_{photo_count:03d}.jpg"
        filepath = os.path.join(SAVE_DIR, filename)
        cv2.imwrite(filepath, snap)
        print(f"[SAVED] photos/{filename}")
        photo_count += 1
        return filename


HTML_PAGE = b"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Camera Capture</title>
<style>
  body {
    margin: 0; background: #111; display: flex; justify-content: center;
    align-items: center; height: 100vh; cursor: pointer; user-select: none;
    font-family: sans-serif;
  }
  img { height: 100vh; object-fit: contain; }
  #hud {
    position: fixed; top: 16px; left: 50%; transform: translateX(-50%);
    background: rgba(0, 0, 0, 0.75); color: #0f0; padding: 8px 16px;
    border-radius: 6px; font-size: 15px; pointer-events: none;
  }
</style>
</head>
<body>
  <div id="hud">Left-click anywhere to save photo</div>
  <img src="/stream" />
  <script>
    const hud = document.getElementById("hud");
    document.addEventListener("mousedown", async (e) => {
      if (e.button !== 0) return; // Left-click only
      const res = await fetch("/snap", { method: "POST" });
      if (res.ok) {
        const name = await res.text();
        hud.textContent = "Saved: " + name;
        hud.style.color = "#00ff88";
      }
    });
  </script>
</body>
</html>
"""


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(HTML_PAGE)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while running:
                    with lock:
                        jpg = latest_jpeg
                    if jpg is None:
                        time.sleep(0.01)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.03)  # ~30 FPS stream cap
            except (BrokenPipeError, ConnectionResetError):
                pass

    def do_POST(self):
        if self.path == "/snap":
            saved = save_snapshot()
            if saved:
                self.send_response(200)
                self.send_header("Content-Type", "text/plain")
                self.end_headers()
                self.wfile.write(saved.encode())
            else:
                self.send_error(503, "No frame available yet")

    def log_message(self, format, *args):
        pass  # Silence HTTP logs so terminal stays clean


def main():
    global running
    threading.Thread(target=capture_loop, daemon=True).start()
    ThreadingHTTPServer.daemon_threads = True
    server = ThreadingHTTPServer(("0.0.0.0", 8000), StreamHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()

    print(f"[SAVE] Photos will be saved to: {SAVE_DIR}")
    print("[LIVE] Open in your laptop browser: http://10.3.141.1:8000 (or http://10.218.139.50:8000)")
    print("[ACTION] Left-click anywhere in the browser window to save a photo | Press Ctrl+C to quit\n")

    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[EXIT] Stopping camera...")
    finally:
        running = False
        server.shutdown()
        picam2.stop()


if __name__ == "__main__":
    main()