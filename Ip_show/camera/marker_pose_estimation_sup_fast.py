#!/usr/bin/env python3
"""ArUco pose estimation on a Raspberry Pi camera - headless & speed-optimised version."""
import argparse
import math
import os
import time

import cv2
import numpy as np

# --- CONFIGURATION ---
ARUCO_DICT_NAME = "DICT_4X4_100"   # Use "DICT_4X4_50" if you only need IDs 0..49
MARKER_SIDE_LENGTH = 0.07          # Exact black-square width in meters
MAX_VALID_DISTANCE = 1.0           # Ignore detections farther than 1.0 m
PRINT_INTERVAL = 0.2               # Terminal print interval (5 Hz)
EXPOSURE_TIME_US = 7500            # Cap exposure (us) so the frame rate doesn't stall in dim light
CAMERA_FPS = 120                   # Requested; libcamera clamps it to what the sensor mode allows
CALIBRATION_FILE = "calibration_chessboard.yaml"   # or pass --calib your_file.yml
FRAME_W, FRAME_H = 1640, 1232      # Capture size. The calibration MUST be made at this size
RES_SCALE = FRAME_W / 640.0        # All pixel-size tunings below were set for 640 px wide frames

# --- SPEED TUNING ---
DETECT_SCALE = 0.5
SUBPIX_HALF_WIN = 6                # Corner refinement half-window (px) at full resolution
ADAPTIVE_WIN_AT_640 = 15
AUTO_MIN_PERIMETER = True
MIN_PERIMETER_SAFETY = 0.6         # < 1.0 leaves room for tilted markers

# ROI tracking: once markers are found, search only around them.
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
    """Euler angles are only computed when printed (not on skipped frames)."""
    R_mat, _ = cv2.Rodrigues(rvec)
    return rotmat_to_euler_xyz_deg(R_mat)


def _odd(x):
    return max(3, int(round(x)) | 1)


def make_detector_params(mtx, image_scale=1.0):
    """ArUco detector parameters tuned for speed (OpenCV 4.6 and 4.7+ compatible)."""
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
            detections.append((int(marker_id), tx, ty, tz, rvec))
        return detections


def main():
    from picamera2 import Picamera2

    parser = argparse.ArgumentParser()
    parser.add_argument("--calib", default=CALIBRATION_FILE,
                        help="calibration .yml/.yaml (relative paths are looked up next to this script)")
    args = parser.parse_args()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    calib_path = args.calib if os.path.isabs(args.calib) else os.path.join(script_dir, args.calib)
    mtx, dst = load_calibration(calib_path)

    if abs(mtx[0, 2] - FRAME_W / 2) > 0.2 * FRAME_W or abs(mtx[1, 2] - FRAME_H / 2) > 0.2 * FRAME_H:
        print(f"[WARN] Calibration principal point ({mtx[0, 2]:.0f}, {mtx[1, 2]:.0f}) is far from the "
              f"centre of a {FRAME_W}x{FRAME_H} frame: was calibration_chessboard.yaml made at this "
              f"resolution? Poses will be wrong otherwise.")

    tracker = MarkerTracker(mtx, dst)
    print(f"[INFO] {FRAME_W}x{FRAME_H}, minMarkerPerimeterRate={tracker.params.minMarkerPerimeterRate:.3f}, "
          f"adaptive window {tracker.params.adaptiveThreshWinSizeMin} (scan: "
          f"{tracker.params_scan.adaptiveThreshWinSizeMin}), detect scale {DETECT_SCALE}, "
          f"ROI tracking={USE_ROI_TRACKING}")

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

    try:
        sensor = picam2.camera_configuration().get("sensor", {})
        frame_us = picam2.capture_metadata().get("FrameDuration")
        print(f"[CAM] sensor mode: {sensor} | frame duration: {frame_us} us "
              f"(~{1e6 / frame_us:.0f} FPS max)" if frame_us else f"[CAM] sensor mode: {sensor}")
    except Exception as exc:
        print(f"[CAM] could not read sensor info: {exc}")

    print(f"[INFO] Tracking '{ARUCO_DICT_NAME}' (size={MARKER_SIDE_LENGTH*100:.1f} cm). Press Ctrl+C to stop.\n")

    last_print = 0.0
    prev_time = time.perf_counter()
    fps = 0.0

    try:
        while True:
            yuv = picam2.capture_array()
            gray = yuv[:FRAME_H, :FRAME_W]  # Direct Y-luminance plane (no CPU RGB->GRAY copy!)

            now = time.perf_counter()
            inst = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            fps = inst if fps == 0.0 else fps + 0.05 * (inst - fps)

            detections = tracker.process(gray)

            if detections and (now - last_print) >= PRINT_INTERVAL:
                last_print = now
                for mid, tx, ty, tz, rvec in detections:
                    roll, pitch, yaw = pose_angles_deg(rvec)
                    print(
                        f"[{fps:5.1f} FPS] ID {mid:02d} | "
                        f"Pos: X={tx:+6.3f}m  Y={ty:+6.3f}m  Z={tz:6.3f}m | "
                        f"Rot: R={roll:+6.1f}°  P={pitch:+6.1f}°  Y={yaw:+6.1f}°"
                    )

    except KeyboardInterrupt:
        print("\n[INFO] Stopping pose estimator...")
    finally:
        picam2.stop()


if __name__ == "__main__":
    main()