#!/usr/bin/env python3
"""ArUco pose estimation on a Raspberry Pi camera - speed-optimised version."""
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
 
import cv2
import numpy as np
 
# --- CONFIGURATION ---
ARUCO_DICT_NAME = "DICT_4X4_100"   # Use "DICT_4X4_50" if you only need IDs 0..49
MARKER_SIDE_LENGTH = 0.07          # Exact black-square width in meters
MAX_VALID_DISTANCE = 1.0           # Ignore detections farther than 1.0 m
ENABLE_WEB_STREAM = True           # View annotated feed at http://10.3.141.1:8000
STREAM_PORT = 8000
STREAM_FPS = 30                    # Max encoding FPS for browser preview
PRINT_INTERVAL = 0.2               # Terminal print interval (5 Hz)
EXPOSURE_TIME_US = 7500            # Cap exposure (us) so the frame rate doesn't stall in dim light
CAMERA_FPS = 120                   # Requested; libcamera clamps it to what the sensor mode allows
CALIBRATION_FILE = "calibration_chessboard.yaml"   # or pass --calib your_file.yml
FRAME_W, FRAME_H = 1640, 1232      # Capture size. The calibration MUST be made at this size
RES_SCALE = FRAME_W / 640.0        # All pixel-size tunings below were set for 640 px wide frames
 
# --- SPEED TUNING ---
# Full-frame scans (looking for new markers) run on a downscaled copy of the image; the
# corners are then refined on the full-resolution image. 1.0 = no downscale.
DETECT_SCALE = 0.5
SUBPIX_HALF_WIN = 6                # corner refinement half-window (px) at full resolution
# One adaptive-threshold pass instead of two. Value is for a 640 px wide frame and is
# scaled automatically to the resolution actually used.
ADAPTIVE_WIN_AT_640 = 15
# Preview stream is downscaled (JPEG-encoding 1640x1232 at 30 FPS would eat the CPU).
STREAM_SCALE = 0.4
STREAM_COLOR = False               # False: grayscale preview (skips the YUV->BGR conversion)
# The minimum marker size is derived from the calibration (focal length), so candidates
# that are too small to be within MAX_VALID_DISTANCE are discarded early.
AUTO_MIN_PERIMETER = True
MIN_PERIMETER_SAFETY = 0.6         # < 1.0 leaves room for tilted markers
# ROI tracking: once markers are found, search only around them. A full-frame scan is
# done every FULL_SCAN_EVERY frames (to discover new markers) and whenever a tracked
# marker is not found in the ROI.
USE_ROI_TRACKING = True
FULL_SCAN_EVERY = 10
ROI_MARGIN = 0.75                  # ROI margin, as a fraction of the marker's pixel size
ROI_MIN_MARGIN_PX = int(round(16 * RES_SCALE))
ROI_MAX_AREA_FRACTION = 0.6        # ROI bigger than this fraction of the frame -> full scan
 
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
 
# Shared state between vision loop and MJPEG encoder thread
latest_jpeg = None
latest_raw_packet = None
jpeg_lock = threading.Lock()
raw_lock = threading.Lock()
stream_clients = 0
running = True
 
 
class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        global stream_clients
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
        elif self.path == "/snapshot.jpg":
            with jpeg_lock:
                jpg = latest_jpeg
            if jpg is None:
                self.send_error(503, "No frame available yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpg)))
            self.end_headers()
            self.wfile.write(jpg)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            stream_clients += 1
            try:
                while running:
                    with jpeg_lock:
                        jpg = latest_jpeg
                    if jpg is None:
                        time.sleep(0.01)
                        continue
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(1.0 / STREAM_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                stream_clients = max(0, stream_clients - 1)
 
    def log_message(self, format, *args):
        pass
 
 
_K_NAMES = ("K", "camera_matrix", "cameraMatrix", "mtx", "intrinsics")
_D_NAMES = ("D", "distortion_coefficients", "dist_coeffs", "distCoeffs", "dist", "dst")
 
 
def load_calibration(yaml_path):
    """Load K and D from an OpenCV FileStorage file (.yml/.yaml), trying common node names."""
    fs = cv2.FileStorage(yaml_path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise FileNotFoundError(f"Calibration file not found: {yaml_path}")
    mtx = dst = None
    for name in _K_NAMES:
        node = fs.getNode(name)
        if not node.empty():
            mtx = node.mat()
            break
    for name in _D_NAMES:
        node = fs.getNode(name)
        if not node.empty():
            dst = node.mat()
            break
    size = []
    for name in ("image_width", "width", "image_height", "height"):
        node = fs.getNode(name)
        if not node.empty():
            size.append((name, int(node.real())))
    fs.release()
    if mtx is None or dst is None:
        raise KeyError(f"{yaml_path}: could not find the camera matrix / distortion nodes "
                       f"(tried {_K_NAMES} and {_D_NAMES})")
    mtx = np.asarray(mtx, dtype=np.float64).reshape(3, 3)
    dst = np.asarray(dst, dtype=np.float64).reshape(-1, 1)
    print(f"[CALIB] {yaml_path}\n        fx={mtx[0, 0]:.1f} fy={mtx[1, 1]:.1f} cx={mtx[0, 2]:.1f} "
          f"cy={mtx[1, 2]:.1f} | {len(dst)} distortion coeffs {dst.ravel().round(5).tolist()}"
          + (f" | stored size {size}" if size else ""))
    for name, val in size:
        expected = FRAME_W if "width" in name else FRAME_H
        if val != expected:
            print(f"[WARN] {name}={val} in the calibration file but the capture is {FRAME_W}x{FRAME_H}")
    if len(dst) == 4:
        print("[WARN] 4 distortion coeffs: if this is a FISHEYE calibration, solvePnP/projectPoints "
              "need cv2.fisheye (or undistortion first); the pinhole model gives wrong poses.")
    return mtx, dst
 
 
def rotmat_to_euler_xyz_deg(R_mat):
    """Fast intrinsic 'xyz' Euler angles (degrees), matching scipy.spatial.transform.Rotation."""
    sy = max(-1.0, min(1.0, float(R_mat[0, 2])))
    pitch = math.asin(sy)
    if abs(sy) < 0.999999:
        roll = math.atan2(-float(R_mat[1, 2]), float(R_mat[2, 2]))
        yaw = math.atan2(-float(R_mat[0, 1]), float(R_mat[0, 0]))
    else:
        roll = math.atan2(float(R_mat[2, 1]), float(R_mat[1, 1]))
        yaw = 0.0
    rad2deg = 180.0 / math.pi
    return roll * rad2deg, pitch * rad2deg, yaw * rad2deg
 
 
def pose_angles_deg(rvec):
    """Euler angles are only computed when they are displayed (not in the hot loop)."""
    R_mat, _ = cv2.Rodrigues(rvec)
    return rotmat_to_euler_xyz_deg(R_mat)
 
 
def draw_labeled_axes(img, mtx, dst, rvec, tvec, length):
    """Draw 3D coordinate axes with X, Y, Z labels at the arrow tips."""
    axis_pts = np.float32([
        [0.0, 0.0, 0.0],
        [length, 0.0, 0.0],
        [0.0, length, 0.0],
        [0.0, 0.0, length],
    ])
    img_pts, _ = cv2.projectPoints(axis_pts, rvec, tvec, mtx, dst)
    pts = np.round(img_pts.reshape(-1, 2)).astype(int)
 
    h, w = img.shape[:2]
    if np.any(np.abs(pts) > max(w, h) * 4):
        return
 
    origin = tuple(pts[0])
    axes = (
        (tuple(pts[1]), (0, 0, 255), "X"),
        (tuple(pts[2]), (0, 255, 0), "Y"),
        (tuple(pts[3]), (255, 80, 0), "Z"),
    )
 
    for tip, color, label in axes:
        cv2.arrowedLine(img, origin, tip, color, 2, cv2.LINE_8, tipLength=0.18)
        cv2.putText(img, label, (tip[0] + 5, tip[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
 
    cv2.circle(img, origin, 3, (255, 255, 255), -1)
 
 
def _odd(x):
    return max(3, int(round(x)) | 1)
 
 
def make_detector_params(mtx, image_scale=1.0):
    """ArUco detector parameters tuned for speed (OpenCV 4.6 and 4.7+ compatible).
 
    image_scale: size of the image the detector will see relative to the full frame.
    """
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        params = cv2.aruco.DetectorParameters_create()
    else:
        params = cv2.aruco.DetectorParameters()
 
    win = _odd(ADAPTIVE_WIN_AT_640 * RES_SCALE * image_scale)
    params.adaptiveThreshWinSizeMin = win
    params.adaptiveThreshWinSizeMax = win
    params.adaptiveThreshWinSizeStep = 10
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementMaxIterations = 10
    params.cornerRefinementMinAccuracy = 0.1
 
    if AUTO_MIN_PERIMETER:
        # Perimeter (px) of a marker seen face-on at MAX_VALID_DISTANCE; anything much
        # smaller would be rejected by the distance filter anyway.
        fx = float(mtx[0, 0])
        perim_px = 4.0 * fx * MARKER_SIDE_LENGTH / MAX_VALID_DISTANCE
        rate = MIN_PERIMETER_SAFETY * perim_px / max(FRAME_W, FRAME_H)
        params.minMarkerPerimeterRate = float(min(max(rate, 0.03), 0.5))
    else:
        params.minMarkerPerimeterRate = 0.08
    return params
 
 
def make_detect_fn(dict_name, params):
    """Return detect(gray) -> (corners, ids), using ArucoDetector when available."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
 
        def detect(img):
            corners, ids, _ = detector.detectMarkers(img)
            return corners, ids
    else:
        def detect(img):
            corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict, parameters=params)
            return corners, ids
    return detect
 
 
class MarkerTracker:
    """Detects markers (full frame or ROI around the last known ones) and estimates poses."""
 
    def __init__(self, mtx, dst):
        self.mtx, self.dst = mtx, dst
        # Two detectors: full-resolution one (used inside ROIs) and one for the
        # downscaled full-frame scans.
        self.params = make_detector_params(mtx, 1.0)
        self.detect = make_detect_fn(ARUCO_DICT_NAME, self.params)
        self.params_scan = make_detector_params(mtx, DETECT_SCALE)
        self.detect_scan = make_detect_fn(ARUCO_DICT_NAME, self.params_scan)
        half = MARKER_SIDE_LENGTH / 2.0
        self.obj_points = np.array(
            [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
            dtype=np.float32,
        )
        self.known_ids = set()
        self.roi = None          # (x0, y0, x1, y1) or None -> full-frame scan
        self.since_full = 0
 
    def _update_roi(self, corners, ids):
        if not USE_ROI_TRACKING or ids is None or len(ids) == 0:
            self.known_ids = set()
            self.roi = None
            return
        self.known_ids = set(ids.ravel().tolist())
        pts = np.concatenate([c.reshape(-1, 2) for c in corners])
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        side = max(float(np.ptp(c.reshape(-1, 2), axis=0).max()) for c in corners)
        m = max(ROI_MIN_MARGIN_PX, ROI_MARGIN * side)
        x0 = max(0, int(x0 - m))
        y0 = max(0, int(y0 - m))
        x1 = min(FRAME_W, int(math.ceil(x1 + m)))
        y1 = min(FRAME_H, int(math.ceil(y1 + m)))
        if (x1 - x0) * (y1 - y0) > ROI_MAX_AREA_FRACTION * FRAME_W * FRAME_H:
            self.roi = None
        else:
            self.roi = (x0, y0, x1, y1)
 
    def detect_markers(self, gray):
        if self.roi is not None and self.since_full < FULL_SCAN_EVERY:
            x0, y0, x1, y1 = self.roi
            corners, ids = self.detect(gray[y0:y1, x0:x1])
            # Accept the ROI result only if every marker we were tracking is still there.
            if ids is not None and self.known_ids.issubset(ids.ravel().tolist()):
                off = np.array([x0, y0], dtype=np.float32)
                corners = tuple(c + off for c in corners)
                self.since_full += 1
                self._update_roi(corners, ids)
                return corners, ids
 
        corners, ids = self.scan_full(gray)
        self.since_full = 0
        self._update_roi(corners, ids)
        return corners, ids
 
    def scan_full(self, gray):
        """Full-frame detection on a downscaled image, corners refined at full resolution."""
        s = DETECT_SCALE
        if s >= 1.0:
            return self.detect(gray)
        small = cv2.resize(gray, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
        corners, ids = self.detect_scan(small)
        if ids is None or len(ids) == 0:
            return corners, ids
        pts = np.concatenate([c.reshape(-1, 2) for c in corners]) / s
        pts = np.ascontiguousarray(pts.reshape(-1, 1, 2), dtype=np.float32)
        cv2.cornerSubPix(
            gray, pts, (SUBPIX_HALF_WIN, SUBPIX_HALF_WIN), (-1, -1),
            (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 10, 0.05),
        )
        corners = tuple(pts[4 * i:4 * i + 4].reshape(1, 4, 2) for i in range(len(ids)))
        return corners, ids
 
    def process(self, gray):
        corners, ids = self.detect_markers(gray)
        detections = []
        if ids is None:
            return detections
 
        for i, marker_id in enumerate(ids.ravel()):
            ok, rvec, tvec = cv2.solvePnP(
                self.obj_points,
                corners[i][0],
                self.mtx,
                self.dst,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not ok:
                continue
            tx, ty, tz = float(tvec[0, 0]), float(tvec[1, 0]), float(tvec[2, 0])
            if not (0.02 < tz < MAX_VALID_DISTANCE):
                continue
            detections.append((int(marker_id), tx, ty, tz, rvec, tvec, corners[i]))
        return detections
 
 
def stream_encoder_worker(mtx, dst, cx, cy):
    """Background thread that converts YUV->BGR, annotates, and encodes JPEGs off the hot loop."""
    global latest_jpeg, latest_raw_packet
    interval = 1.0 / STREAM_FPS
    # Intrinsics and image centre for the downscaled preview
    mtx_s = np.array(mtx, dtype=np.float64).copy()
    mtx_s[:2] *= STREAM_SCALE
    cx, cy = int(round(cx * STREAM_SCALE)), int(round(cy * STREAM_SCALE))
 
    while running:
        if stream_clients == 0 and latest_jpeg is not None:
            time.sleep(0.05)
            continue
 
        with raw_lock:
            packet = latest_raw_packet
            latest_raw_packet = None
 
        if packet is None:
            time.sleep(0.005)
            continue
 
        yuv_frame, detections, fps, roi = packet
        # yuv_frame rows may be padded (stride > FRAME_W): crop before resizing.
        if STREAM_COLOR:
            full = cv2.cvtColor(yuv_frame, cv2.COLOR_YUV2BGR_I420)[:FRAME_H, :FRAME_W]
        else:
            full = cv2.cvtColor(yuv_frame[:FRAME_H, :FRAME_W], cv2.COLOR_GRAY2BGR)
        vis = cv2.resize(full, None, fx=STREAM_SCALE, fy=STREAM_SCALE, interpolation=cv2.INTER_AREA)
        sx = STREAM_SCALE
 
        cv2.line(vis, (cx - 12, cy), (cx + 12, cy), (180, 180, 180), 1)
        cv2.line(vis, (cx, cy - 12), (cx, cy + 12), (180, 180, 180), 1)
        cv2.putText(vis, f"{fps:.1f} FPS", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        if roi is not None:
            cv2.rectangle(vis, (int(roi[0] * sx), int(roi[1] * sx)),
                          (int(roi[2] * sx), int(roi[3] * sx)), (255, 255, 0), 1)
 
        for idx, (mid, tx, ty, tz, rvec, tvec, c) in enumerate(detections):
            roll, pitch, yaw = pose_angles_deg(rvec)
            cv2.aruco.drawDetectedMarkers(vis, [(c * sx).astype(np.float32)], np.array([[mid]]))
            draw_labeled_axes(vis, mtx_s, dst, rvec, tvec, MARKER_SIDE_LENGTH * 0.75)
            hud = f"ID {mid:02d}: X={tx:+.2f} Y={ty:+.2f} Z={tz:.2f}m | R={roll:+.0f} P={pitch:+.0f} Y={yaw:+.0f}"
            cv2.putText(vis, hud, (10, 50 + idx * 24), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 255), 2)
 
        ok, buf = cv2.imencode(".jpg", vis, [cv2.IMWRITE_JPEG_QUALITY, 75])
        if ok:
            with jpeg_lock:
                latest_jpeg = buf.tobytes()
 
        time.sleep(interval)
 
 
def main():
    global latest_raw_packet, running
    from picamera2 import Picamera2  # imported here so the module can be tested without a camera
 
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--calib", default=CALIBRATION_FILE,
                        help="calibration .yml/.yaml (relative paths are looked up next to this script)")
    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = args.calib if os.path.isabs(args.calib) else os.path.join(script_dir, args.calib)
    mtx, dst = load_calibration(calib_path)
    cx, cy = int(round(mtx[0, 2])), int(round(mtx[1, 2]))
 
    if abs(mtx[0, 2] - FRAME_W / 2) > 0.2 * FRAME_W or abs(mtx[1, 2] - FRAME_H / 2) > 0.2 * FRAME_H:
        print(f"[WARN] Calibration principal point ({mtx[0, 2]:.0f}, {mtx[1, 2]:.0f}) is far from the "
              f"centre of a {FRAME_W}x{FRAME_H} frame: was calibration_chessboard.yaml made at this "
              f"resolution? Poses will be wrong otherwise.")
 
    tracker = MarkerTracker(mtx, dst)
    print(f"[INFO] {FRAME_W}x{FRAME_H}, minMarkerPerimeterRate={tracker.params.minMarkerPerimeterRate:.3f}, "
          f"adaptive window {tracker.params.adaptiveThreshWinSizeMin} (scan: "
          f"{tracker.params_scan.adaptiveThreshWinSizeMin}), detect scale {DETECT_SCALE}, "
          f"ROI tracking={USE_ROI_TRACKING}")
 
    # Web stream server + background JPEG encoder thread
    server = None
    if ENABLE_WEB_STREAM:
        server = ThreadingHTTPServer(("0.0.0.0", STREAM_PORT), StreamHandler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
        threading.Thread(target=stream_encoder_worker, args=(mtx, dst, cx, cy), daemon=True).start()
        print(f"[STREAM] Live preview: http://10.3.141.1:{STREAM_PORT}")
 
    # Camera initialization in native YUV420 (zero-copy grayscale plane)
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(
        main={"size": (FRAME_W, FRAME_H), "format": "YUV420"},
        buffer_count=2,
        controls={"FrameRate": CAMERA_FPS, "ExposureTime": EXPOSURE_TIME_US},
    )
    picam2.configure(config)
    picam2.start()
    time.sleep(0.2)
 
    # Check that the camera really runs in a fast sensor mode (a slow mode caps the FPS).
    try:
        sensor = picam2.camera_configuration().get("sensor", {})
        frame_us = picam2.capture_metadata().get("FrameDuration")
        print(f"[CAM] sensor mode: {sensor} | frame duration: {frame_us} us "
              f"(~{1e6 / frame_us:.0f} FPS max)" if frame_us else f"[CAM] sensor mode: {sensor}")
    except Exception as exc:  # purely informative
        print(f"[CAM] could not read sensor info: {exc}")
 
    print(f"[INFO] Tracking '{ARUCO_DICT_NAME}' (size={MARKER_SIDE_LENGTH*100:.1f} cm). Press Ctrl+C to stop.\n")
 
    last_print = 0.0
    prev_time = time.perf_counter()
    fps = 0.0
 
    try:
        while True:
            # capture_array() already returns a private copy, so it can be shared with the
            # encoder thread without another copy.
            yuv = picam2.capture_array()
            gray = yuv[:FRAME_H, :FRAME_W]  # Direct Y-luminance plane (no CPU RGB->GRAY copy!)
 
            now = time.perf_counter()
            inst = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            fps = inst if fps == 0.0 else fps + 0.05 * (inst - fps)  # smoothed FPS
 
            detections = tracker.process(gray)
 
            if detections and (now - last_print) >= PRINT_INTERVAL:
                last_print = now
                for mid, tx, ty, tz, rvec, _, _ in detections:
                    roll, pitch, yaw = pose_angles_deg(rvec)
                    print(
                        f"[{fps:5.1f} FPS] ID {mid:02d} | "
                        f"Pos: X={tx:+6.3f}m  Y={ty:+6.3f}m  Z={tz:6.3f}m | "
                        f"Rot: R={roll:+6.1f}°  P={pitch:+6.1f}°  Y={yaw:+6.1f}°"
                    )
 
            # Hand the frame to the encoder thread only when someone is watching
            if ENABLE_WEB_STREAM and (stream_clients > 0 or latest_jpeg is None):
                with raw_lock:
                    if latest_raw_packet is None:
                        latest_raw_packet = (yuv, detections, fps, tracker.roi)
 
    except KeyboardInterrupt:
        print("\n[INFO] Stopping pose estimator...")
    finally:
        running = False
        if server is not None:
            server.shutdown()
        picam2.stop()
 
 
if __name__ == "__main__":
    main()
 