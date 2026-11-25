import math
import time
import threading
from statistics import median
import numpy as np
from rplidar import RPLidar, RPLidarException

# ==============================================================================
# 1. CALIBRATION & TABLE GEOMETRY (from lidar_table.py, converted to meters)
# ==============================================================================
PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

BEACON_RADIUS_M = 0.040        # 40 mm radius (80 mm diameter Eurobot beacon)
DIST_SCALE = 1.0               # Multiplicative range gain
DIST_OFFSET_M = -0.210         # Constant range offset (-210 mm)

LIDAR_ANGLE_SIGN = -1.0        # RPLIDAR CW -> standard robot CCW (+Y left)
LIDAR_YAW_OFFSET_RAD = math.radians(0.0)
# Optional lever-arm offset of the LiDAR center relative to the axle midpoint (m)
LIDAR_OFFSET_X_M = 0.0
LIDAR_OFFSET_Y_M = 0.0

TABLE_LENGTH_M = 3.000
TABLE_WIDTH_M = 2.000
START_X_IN_TABLE_M = 0.225     # Yellow start center X (225 mm)
START_Y_IN_TABLE_M = 1.000     # Yellow start center Y (1000 mm)
WALL_MARGIN_M = 0.045
BEACON_POCKET_RADIUS_M = 0.180

# Fixed beacon coordinates in world frame {W} where Yellow Start Center = (0, 0)
KNOWN_BEACONS_M = {
    "Back_Right": (-0.062 - START_X_IN_TABLE_M, -0.062 - START_Y_IN_TABLE_M),
    "Back_Left":  (-0.062 - START_X_IN_TABLE_M, (TABLE_WIDTH_M + 0.062) - START_Y_IN_TABLE_M),
    "Opp_Middle": ((TABLE_LENGTH_M + 0.062) - START_X_IN_TABLE_M, 1.000 - START_Y_IN_TABLE_M),
}

MIN_PLAY_X = -START_X_IN_TABLE_M + WALL_MARGIN_M
MAX_PLAY_X = (TABLE_LENGTH_M - START_X_IN_TABLE_M) - WALL_MARGIN_M
MIN_PLAY_Y = -START_Y_IN_TABLE_M + WALL_MARGIN_M
MAX_PLAY_Y = (TABLE_WIDTH_M - START_Y_IN_TABLE_M) - WALL_MARGIN_M

# Clustering thresholds (meters)
MIN_DIST_M = 0.100
MAX_DIST_M = 3.600
MAX_GAP_M = 0.075
MAX_BEACON_WIDTH_M = 0.140
MAX_RADIAL_SPREAD_M = 0.085


def wrap_angle(angle_rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class EKFLocalizer:
    """
    Extended Kalman Filter fusing passive dead-wheel odometry and 2D LiDAR beacons.
    Follows LELME2732 Lecture 4 (Slides 18-20 & 48-55).
    """

    def __init__(
        self,
        wheel_base_m: float = 0.240,        # ODO_WHEEL_BASE_M from PID.py
        init_pose=(0.0, 0.0, 0.0),          # (x [m], y [m], theta [rad])
        init_std=(0.015, 0.015, math.radians(2.0)),
        kr: float = 0.002,                  # Right wheel variance per meter traveled [m^2/m]
        kl: float = 0.002,                  # Left wheel variance per meter traveled [m^2/m]
        sigma_range_m: float = 0.025,       # LiDAR range std dev [m]
        sigma_bearing_rad: float = math.radians(1.5),  # LiDAR bearing std dev [rad]
        mahalanobis_gate_sq: float = 9.21,  # Chi-square 2 DOF at 99% confidence (Slide 54)
    ):
        self.b = wheel_base_m
        self.kr = kr
        self.kl = kl
        self.R = np.diag([sigma_range_m ** 2, sigma_bearing_rad ** 2])
        self.gate_sq = mahalanobis_gate_sq

        self.x = np.array(init_pose, dtype=float)
        self.P = np.diag([init_std[0] ** 2, init_std[1] ** 2, init_std[2] ** 2])
        self._lock = threading.Lock()

        # Stats for diagnostics
        self.last_matched_beacons = {}
        self.last_lidar_update_time = 0.0

    def get_pose(self):
        """Returns thread-safe (x_m, y_m, theta_rad) and covariance P."""
        with self._lock:
            return float(self.x[0]), float(self.x[1]), float(self.x[2]), self.P.copy()

    def set_pose(self, x_m: float, y_m: float, theta_rad: float, std=(0.01, 0.01, math.radians(1.5))):
        with self._lock:
            self.x = np.array([x_m, y_m, wrap_angle(theta_rad)], dtype=float)
            self.P = np.diag([std[0] ** 2, std[1] ** 2, std[2] ** 2])

    # --------------------------------------------------------------------------
    # STEP 0: PREDICTION UPDATE (Lecture 4, Slides 18-20 & Slide 48)
    # --------------------------------------------------------------------------
    def predict_odometry(self, ds_left: float, ds_right: float):
        """
        Propagates state x_hat and covariance P_hat from incremental wheel displacements.
        ds_left, ds_right: incremental distances [m] of the outer passive wheels.
        """
        if abs(ds_left) < 1e-9 and abs(ds_right) < 1e-9:
            return

        with self._lock:
            x, y, theta = self.x

            ds = 0.5 * (ds_right + ds_left)
            dtheta = (ds_right - ds_left) / self.b
            theta_mid = theta + 0.5 * dtheta

            c = math.cos(theta_mid)
            s = math.sin(theta_mid)

            # Motion model f(x, y, theta, ds_r, ds_l) (Slide 18)
            x_hat = x + ds * c
            y_hat = y + ds * s
            theta_hat = wrap_angle(theta + dtheta)

            # Jacobian wrt pose F_x = nabla_p f (Slide 20)
            Fx = np.array([
                [1.0, 0.0, -ds * s],
                [0.0, 1.0,  ds * c],
                [0.0, 0.0,  1.0],
            ], dtype=float)

            # Jacobian wrt wheel increments F_u = nabla_{Delta rl} f (Slide 20)
            Fu = np.array([
                [0.5 * c - (ds / (2.0 * self.b)) * s,  0.5 * c + (ds / (2.0 * self.b)) * s],
                [0.5 * s + (ds / (2.0 * self.b)) * c,  0.5 * s - (ds / (2.0 * self.b)) * c],
                [1.0 / self.b,                        -1.0 / self.b],
            ], dtype=float)

            # Odometry error covariance Q_t = Sigma_Delta (Slide 19)
            Q = np.array([
                [self.kr * abs(ds_right), 0.0],
                [0.0, self.kl * abs(ds_left)],
            ], dtype=float)

            # Covariance propagation: P_hat = Fx * P * Fx^T + Fu * Q * Fu^T (Slide 48)
            self.x = np.array([x_hat, y_hat, theta_hat], dtype=float)
            self.P = Fx @ self.P @ Fx.T + Fu @ Q @ Fu.T

    # --------------------------------------------------------------------------
    # STEPS 2, 3, 4: MEASUREMENT PREDICTION, GATING & FUSION (Slides 51-55)
    # --------------------------------------------------------------------------
    def update_beacons(self, observations: list):
        """
        observations: list of dicts with {'range_m': r_i, 'bearing_rad': phi_i} in robot frame.
        """
        if not observations:
            return

        with self._lock:
            matched_summary = {}
            used_beacons = set()

            for obs in observations:
                z = np.array([obs['range_m'], obs['bearing_rad']], dtype=float)

                best_name = None
                best_d2 = self.gate_sq
                best_innovation = None
                best_H = None
                best_S = None

                # Current state estimate (sequential EKF update)
                rx, ry, rtheta = self.x
                c_r, s_r = math.cos(rtheta), math.sin(rtheta)
                # LiDAR world position accounting for optional mounting offset
                sx = rx + (LIDAR_OFFSET_X_M * c_r - LIDAR_OFFSET_Y_M * s_r)
                sy = ry + (LIDAR_OFFSET_X_M * s_r + LIDAR_OFFSET_Y_M * c_r)

                # Step 2 & 3: Predict measurement for each map beacon and test Mahalanobis gate
                for name, (bx, by) in KNOWN_BEACONS_M.items():
                    if name in used_beacons:
                        continue

                    dx = bx - sx
                    dy = by - sy
                    q = dx * dx + dy * dy
                    r_hat = math.sqrt(q)
                    if r_hat < 1e-6:
                        continue

                    phi_hat = wrap_angle(math.atan2(dy, dx) - rtheta)
                    z_hat = np.array([r_hat, phi_hat], dtype=float)

                    # Innovation v_t^{ij} = z_t^i - z_hat_t^j (Slide 53)
                    v = np.array([z[0] - z_hat[0], wrap_angle(z[1] - z_hat[1])], dtype=float)

                    # Measurement Jacobian H_j = dh_j / dx
                    H = np.array([
                        [-dx / r_hat, -dy / r_hat, 0.0],
                        [ dy / q,     -dx / q,    -1.0],
                    ], dtype=float)

                    # Innovation covariance Sigma_IN = H * P_hat * H^T + R (Slide 53)
                    S = H @ self.P @ H.T + self.R
                    try:
                        S_inv = np.linalg.inv(S)
                    except np.linalg.LinAlgError:
                        continue

                    # Squared Mahalanobis distance d_M^2 = v^T * S^-1 * v (Slide 54)
                    d2 = float(v.T @ S_inv @ v)
                    if d2 < best_d2:
                        best_d2 = d2
                        best_name = name
                        best_innovation = v
                        best_H = H
                        best_S = S_inv

                # Step 4: Kalman gain & State/Covariance update (Slide 55)
                if best_name is not None:
                    used_beacons.add(best_name)
                    K = self.P @ best_H.T @ best_S
                    self.x = self.x + K @ best_innovation
                    self.x[2] = wrap_angle(self.x[2])

                    # Joseph form update for numerical positive-definiteness
                    I_KH = np.eye(3) - K @ best_H
                    self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T

                    matched_summary[best_name] = {
                        'range_m': obs['range_m'],
                        'bearing_deg': math.degrees(obs['bearing_rad']),
                        'mahalanobis_d2': best_d2,
                    }

            self.last_matched_beacons = matched_summary
            if matched_summary:
                self.last_lidar_update_time = time.time()


# ==============================================================================
# 2. STEP 1: FEATURE EXTRACTION FROM RAW LIDAR SCANS (Lecture 4, Slides 49-50)
# ==============================================================================
def _is_inside_valid_zone(wx: float, wy: float) -> bool:
    if MIN_PLAY_X <= wx <= MAX_PLAY_X and MIN_PLAY_Y <= wy <= MAX_PLAY_Y:
        return True
    for bx, by in KNOWN_BEACONS_M.values():
        if math.hypot(wx - bx, wy - by) <= BEACON_POCKET_RADIUS_M:
            return True
    return False


def extract_beacon_observations(scan, predicted_pose) -> list:
    """
    Filters out spectators outside the 3m x 2m table using the predicted EKF pose,
    clusters cylinder arcs, and returns polar observations [{'range_m', 'bearing_rad'}].
    """
    rx, ry, rtheta = predicted_pose
    c, s = math.cos(rtheta), math.sin(rtheta)

    points = []
    for quality, angle_deg, dist_mm in scan:
        dist_m = dist_mm * 0.001
        if quality <= 0 or not (MIN_DIST_M <= dist_m <= MAX_DIST_M):
            continue

        rel_rad = wrap_angle(LIDAR_ANGLE_SIGN * math.radians(angle_deg) + LIDAR_YAW_OFFSET_RAD)
        lx = dist_m * math.cos(rel_rad) + LIDAR_OFFSET_X_M
        ly = dist_m * math.sin(rel_rad) + LIDAR_OFFSET_Y_M

        wx = rx + (lx * c - ly * s)
        wy = ry + (lx * s + ly * c)
        if not _is_inside_valid_zone(wx, wy):
            continue

        points.append({'rad': rel_rad, 'dist': dist_m, 'lx': lx, 'ly': ly})

    if not points:
        return []

    points.sort(key=lambda p: p['rad'])

    # Cluster contiguous points
    clusters = []
    curr = [points[0]]
    for p in points[1:]:
        if math.hypot(p['lx'] - curr[-1]['lx'], p['ly'] - curr[-1]['ly']) <= MAX_GAP_M:
            curr.append(p)
        else:
            clusters.append(curr)
            curr = [p]
    clusters.append(curr)

    if len(clusters) > 1:
        if math.hypot(clusters[0][0]['lx'] - clusters[-1][-1]['lx'],
                      clusters[0][0]['ly'] - clusters[-1][-1]['ly']) <= MAX_GAP_M:
            clusters[0].extend(clusters.pop())

    # Fit cylindrical beacon centers
    observations = []
    for cluster in clusters:
        if len(cluster) < 2:
            continue

        front_ref = min(p['dist'] for p in cluster)
        inliers = [p for p in cluster if (p['dist'] - front_ref) <= MAX_RADIAL_SPREAD_M]
        if len(inliers) < 2:
            continue

        width = math.hypot(inliers[0]['lx'] - inliers[-1]['lx'], inliers[0]['ly'] - inliers[-1]['ly'])
        radial_spread = max(p['dist'] for p in inliers) - front_ref

        if width <= MAX_BEACON_WIDTH_M and radial_spread <= MAX_RADIAL_SPREAD_M:
            raw_dist = median(p['dist'] for p in inliers)
            cal_range_m = (raw_dist * DIST_SCALE) + DIST_OFFSET_M + BEACON_RADIUS_M

            ux = sum(math.cos(p['rad']) for p in inliers)
            uy = sum(math.sin(p['rad']) for p in inliers)
            bearing_rad = math.atan2(uy, ux)

            observations.append({
                'range_m': cal_range_m,
                'bearing_rad': bearing_rad,
            })

    return observations


# ==============================================================================
# 3. BACKGROUND LIDAR WORKER THREAD
# ==============================================================================
class LidarLocalizationThread(threading.Thread):
    def __init__(self, ekf: EKFLocalizer, port: str = PORT_NAME, baudrate: int = BAUDRATE):
        super().__init__(daemon=True)
        self.ekf = ekf
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.lidar = None

    def _connect(self, retries: int = 3):
        lidar = RPLidar(self.port, baudrate=self.baudrate, timeout=3)
        lidar.stop()
        lidar.clear_input()
        for _ in range(retries):
            try:
                print(f"[LiDAR] Connected: {lidar.get_info()}")
                return lidar
            except RPLidarException as e:
                print(f"[LiDAR WARN] get_info failed ({e}), resetting...")
                lidar.reset()
                time.sleep(2.0)
                lidar.clear_input()
        raise RuntimeError("RPLIDAR unresponsive")

    def run(self):
        self.running = True
        try:
            self.lidar = self._connect()
            self.lidar.start_motor()
            time.sleep(1.5)

            for scan in self.lidar.iter_scans():
                if not self.running:
                    break
                rx, ry, rtheta, _ = self.ekf.get_pose()
                obs = extract_beacon_observations(scan, (rx, ry, rtheta))
                if obs:
                    self.ekf.update_beacons(obs)

        except RPLidarException as e:
            print(f"[LiDAR ERROR] {e}")
        finally:
            self.stop()

    def stop(self):
        self.running = False
        if self.lidar:
            for fn in (self.lidar.stop, self.lidar.stop_motor, self.lidar.disconnect):
                try:
                    fn()
                except Exception:
                    pass
            self.lidar = None