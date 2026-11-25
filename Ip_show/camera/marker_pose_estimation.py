#!/usr/bin/env python3
# Tracks ArUco markers with a Raspberry Pi camera and reports their 3D pose.
# Standard-library tools for file paths, threading, timing, and the HTTP server.
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
# OpenCV: image processing, ArUco marker detection, and pose estimation.
import cv2
# NumPy: arrays and 3D coordinates used by OpenCV.
import numpy as np
# Raspberry Pi camera interface.
from picamera2 import Picamera2
# Converts rotation matrices into readable roll, pitch, and yaw angles.
from scipy.spatial.transform import Rotation as R

# --- CONFIGURATION ---
# Adjust these settings to match the printed marker and desired output.
ARUCO_DICT_NAME = "DICT_4X4_100"   # Strict 4x4 dictionary (much fewer false positives)
MARKER_SIDE_LENGTH = 0.07       # Exact black-square width in meters (measure your print!)
MAX_VALID_DISTANCE = 1.0          # Ignore false detections farther than 1 meter
ENABLE_WEB_STREAM = True          # View annotated feed at http://10.3.141.1:8000
STREAM_PORT = 8000
PRINT_INTERVAL = 0.2              # Seconds between terminal prints (5 Hz)

# Available ArUco dictionaries. ARUCO_DICT_NAME above selects one of these.
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}

# Shared state for the browser preview and snapshot. The lock protects the JPEG
# while the camera thread updates it and web-server threads read it.
latest_jpeg = None
stream_lock = threading.Lock()
stream_clients = 0
running = True


# Handles browser requests for the video page, live stream, and still snapshot.
class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # The HTTP server calls this method for each incoming GET request.
        global stream_clients
        # Home page: show the axis legend and embed the live MJPEG stream.
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b'<html><body style="margin:0;background:#111;color:#eee;font-family:sans-serif;'
                b'display:flex;flex-direction:column;align-items:center;justify-content:center;height:100vh;">'
                b'<div style="padding:8px;font-size:14px;">'
                b'<b>Marker Axes:</b> <span style="color:#ff5555;">Red = +X (Right)</span> | '
                b'<span style="color:#55ff55;">Green = +Y (Up)</span> | '
                b'<span style="color:#5599ff;">Blue = +Z (Out)</span> &nbsp;&nbsp;'
                b'<a href="/snapshot.jpg" target="_blank" style="color:#ffd700;">[Open Snapshot]</a>'
                b'</div>'
                b'<img src="/stream" style="max-height:90vh;border:1px solid #333;"/>'
                b'</body></html>'
            )
        # Snapshot endpoint: return the latest frame as a normal JPEG image.
        elif self.path == "/snapshot.jpg":
            with stream_lock:
                jpg = latest_jpeg
            if jpg is None:
                self.send_error(503, "No frame available yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        # Stream endpoint: continuously send JPEG frames in MJPEG format.
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            stream_clients += 1
            try:
                while running:
                    with stream_lock:
                        jpg = latest_jpeg
                    if jpg is None:
                        time.sleep(0.01)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(0.033)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                stream_clients = max(0, stream_clients - 1)

    # Suppress the HTTP server's routine request logs in the terminal.
    def log_message(self, format, *args):
        pass


# Load camera calibration values from a YAML file.
# K is the camera matrix; D contains lens-distortion coefficients.
def load_calibration(yaml_path):
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Calibration file not found: {yaml_path}")
    mtx = fs.getNode("K").mat()
    dst = fs.getNode("D").mat()
    fs.release()
    return mtx, dst


# Draw the marker's local X, Y, and Z axes on the camera image.
# rvec/tvec describe the marker's rotation and translation relative to the camera.
def draw_labeled_axes(img, mtx, dst, rvec, tvec, length):
    """Draw 3D coordinate axes with X, Y, Z labels at the arrow tips."""
    axis_pts = np.float32([
        [0.0, 0.0, 0.0],       # Origin (marker center)
        [length, 0.0, 0.0],    # +X (Red)
        [0.0, length, 0.0],    # +Y (Green)
        [0.0, 0.0, length],    # +Z (Blue)
    ])
    img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, mtx, dst)
    pts = np.round(img_pts.reshape(-1, 2)).astype(int)

    h, w = img.shape[:2]
    if np.any(np.abs(pts) > max(w, h) * 4):
        return  # Guard against extreme off-screen projections

    origin = tuple(pts[0])
    axes = [
        (tuple(pts[1]), (0, 0, 255), "X"),   # BGR Red
        (tuple(pts[2]), (0, 255, 0), "Y"),   # BGR Green
        (tuple(pts[3]), (255, 80, 0), "Z"),  # BGR Blue
    ]

    for tip, color, label in axes:
        cv2.arrowedLine(img, origin, tip, color, 3, cv2.LINE_AA, tipLength=0.18)
        cv2.putText(img, label, (tip[0] + 5, tip[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.65, color, 2, cv2.LINE_AA)

    cv2.circle(img, origin, 4, (255, 255, 255), -1)


# Main entry point: configure calibration, detector, web preview, and camera tracking.
def main():
    global latest_jpeg, running

    # Locate the calibration file in the same folder as this script.
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = os.path.join(script_dir, "calibration_chessboard.yaml")
    mtx, dst = load_calibration(calib_path)
    cx, cy = int(round(mtx[0, 2])), int(round(mtx[1, 2]))

    # 1. Configure the ArUco detector. Sub-pixel refinement estimates marker corners
    # more precisely, which can improve the pose calculation.
    dict_id = ARUCO_DICT[ARUCO_DICT_NAME]
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        aruco_params = cv2.aruco.DetectorParameters_create()
    else:
        aruco_params = cv2.aruco.DetectorParameters()

    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # 3D coordinates of the four marker corners, expressed in the marker's own frame
    # and measured in metres. Their order must match OpenCV's detected corner order.
    # Corner order from OpenCV: Top-Left, Top-Right, Bottom-Right, Bottom-Left
    # -> +X points Right, +Y points Up, +Z points Out toward camera (Right-Hand Rule)
    half = MARKER_SIDE_LENGTH / 2.0
    obj_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float32,
    )

    # 2. Start the optional web server in a background thread so tracking can continue.
    server = None
    if ENABLE_WEB_STREAM:
        server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), StreamHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        print(f"[STREAM] Live preview: http://10.3.141.1:{STREAM_PORT}")

    # 3. Initialize the Raspberry Pi camera. Use a resolution compatible with the
    # calibration data; calibration may need adjustment if the image size changes.
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (1640, 1232), "format": "RGB888"},
        buffer_count=4,
        controls={"FrameRate": 120},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)

    print(f"[INFO] Tracking '{ARUCO_DICT_NAME}' (size={MARKER_SIDE_LENGTH*100:.1f} cm). Press Ctrl+C to stop.\n")

    # Timestamp of the last terminal update, used to limit how often data is printed.
    last_print = 0.0
    prev_time = time.perf_counter()

    try:
        while True:
            # Capture a frame, then calculate the approximate processing frame rate (FPS).
            frame = picam2.capture_array()
            now = time.perf_counter()
            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now

            # Detect markers. 'corners' stores their four image corners; 'marker_ids'
            # stores the ID associated with each detected marker.
            corners, marker_ids, _ = cv2.aruco.detectMarkers(
                frame, aruco_dict, parameters=aruco_params
            )

            # Keep only detections with a valid pose and a plausible distance.
            detections = []
            if marker_ids is not None:
                for i, marker_id in enumerate(marker_ids.flatten()):
                    # Estimate the marker's 3D pose from its known 3D corners and detected
                    # 2D image corners. rvec is rotation; tvec is position in camera space.
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points,
                        corners[i][0],
                        mtx,
                        dst,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    )
                    if not ok:
                        continue

                    tx, ty, tz = tvec.flatten()
                    if not (0.02 < tz < MAX_VALID_DISTANCE):
                        continue  # Reject physically implausible outliers

                    # Convert the rotation vector to a matrix, then extract roll, pitch,
                    # and yaw in degrees using the XYZ Euler-angle convention.
                    rot_mat, _ = cv2.Rodrigues(rvec)
                    roll, pitch, yaw = R.from_matrix(rot_mat).as_euler("xyz", degrees=True)
                    detections.append((int(marker_id), tx, ty, tz, roll, pitch, yaw, rvec, tvec, corners[i]))

            # Print valid marker measurements no more often than PRINT_INTERVAL.
            if detections and (now - last_print) >= PRINT_INTERVAL:
                last_print = now
                for mid, tx, ty, tz, roll, pitch, yaw, _, _, _ in detections:
                    print(
                        f"[{fps:4.1f} FPS] ID {mid:02d} | "
                        f"Pos: X={tx:+6.3f}m  Y={ty:+6.3f}m  Z={tz:6.3f}m | "
                        f"Rot: R={roll:+6.1f}°  P={pitch:+6.1f}°  Y={yaw:+6.1f}°"
                    )

            # Draw and encode a frame only when a browser is connected or no snapshot
            # frame exists yet. This avoids JPEG-encoding every camera frame unnecessarily.
            if ENABLE_WEB_STREAM and (stream_clients > 0 or latest_jpeg is None):
                vis = frame.copy()

                # Draw a crosshair at the camera's calibrated optical centre.
                cv2.line(vis, (cx - 12, cy), (cx + 12, cy), (180, 180, 180), 1)
                cv2.line(vis, (cx, cy - 12), (cx, cy + 12), (180, 180, 180), 1)

                cv2.putText(vis, f"{fps:.1f} FPS", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

                for idx, (mid, tx, ty, tz, roll, pitch, yaw, rvec, tvec, c) in enumerate(detections):
                    cv2.aruco.drawDetectedMarkers(vis, [c], np.array([[mid]]))
                    draw_labeled_axes(vis, mtx, dst, rvec, tvec, MARKER_SIDE_LENGTH * 0.75)

                    # Display this marker's position and orientation as an on-screen readout.
                    hud = f"ID {mid:02d}: X={tx:+.2f} Y={ty:+.2f} Z={tz:.2f}m | R={roll:+.0f} P={pitch:+.0f} Y={yaw:+.0f}"
                    cv2.putText(vis, hud, (10, 50 + idx * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)

                ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok:
                    with stream_lock:
                        latest_jpeg = buf.tobytes()

    # Handle Ctrl+C and make sure resources are released below.
    except KeyboardInterrupt:
        print("\n[INFO] Stopping pose estimator...")
    finally:
        running = False
        if server is not None:
            server.shutdown()
        picam2.stop()


# Run main() only when this file is executed directly, not when imported by another file.
if __name__ == "__main__":
    main()