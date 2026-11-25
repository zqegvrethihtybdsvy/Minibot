import time
import math
import logging
from statistics import median
from rplidar import RPLidar, RPLidarException

logging.basicConfig(level=logging.WARNING)

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

# --- RANGE CALIBRATION (tune with 1.6 m test) ---
BEACON_RADIUS_MM = 40.0     # 40 mm for an 80 mm diameter Eurobot beacon
DIST_SCALE = 1.0            # Multiplicative gain calibration
DIST_OFFSET_MM = -210.0     # Constant offset in mm (adjust so 1.6 m true reads 1600 mm)

# --- LIDAR MOUNTING ORIENTATION ---
# RPLIDAR angles increase clockwise; standard robot frame (+X forward, +Y left) is counter-clockwise (-1.0)
LIDAR_ANGLE_SIGN = -1.0
LIDAR_YAW_OFFSET_DEG = 0.0  # Mounting angle offset of the LiDAR's 0° mark relative to robot +X

# --- TABLE & START GEOMETRY (mm) ---
TABLE_LENGTH_MM = 3000.0
TABLE_WIDTH_MM = 2000.0
START_X_IN_TABLE_MM = 225.0   # Middle of the yellow starting zone along the 3m axis
START_Y_IN_TABLE_MM = 1000.0  # Middle of the yellow starting zone along the 2m axis
WALL_MARGIN_MM = 45.0         # Ignore border walls and people leaning near the edges
BEACON_POCKET_RADIUS_MM = 180.0
BEACON_MATCH_GATE_MM = 350.0  # Max distance between predicted and detected beacon to associate

# Known beacon positions converted into the user's frame where Yellow Start Center = (0, 0)
# Standard Eurobot supports sit ~62 mm outside the 3000x2000 inner walls
KNOWN_BEACONS = {
    "Back_Right": (-62.0 - START_X_IN_TABLE_MM, -62.0 - START_Y_IN_TABLE_MM),
    "Back_Left":  (-62.0 - START_X_IN_TABLE_MM, (TABLE_WIDTH_MM + 62.0) - START_Y_IN_TABLE_MM),
    "Opp_Middle": ((TABLE_LENGTH_MM + 62.0) - START_X_IN_TABLE_MM, 1000.0 - START_Y_IN_TABLE_MM),
}

# Inner table bounds in the user's (0, 0)-at-start coordinate frame
MIN_PLAY_X = -START_X_IN_TABLE_MM + WALL_MARGIN_MM
MAX_PLAY_X = (TABLE_LENGTH_MM - START_X_IN_TABLE_MM) - WALL_MARGIN_MM
MIN_PLAY_Y = -START_Y_IN_TABLE_MM + WALL_MARGIN_MM
MAX_PLAY_Y = (TABLE_WIDTH_MM - START_Y_IN_TABLE_MM) - WALL_MARGIN_MM

# --- CLUSTERING PARAMETERS ---
MIN_DIST_MM = 100
MAX_DIST_MM = 3600          # Needs > 3.1 m to reach the opposite middle beacon from start
MAX_GAP_MM = 75
MAX_BEACON_WIDTH_MM = 140
MAX_RADIAL_SPREAD_MM = 85


def connect_lidar(retries=3):
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=3)
    lidar.stop()
    lidar.clear_input()
    for _ in range(retries):
        try:
            print(f"[LiDAR] Info : {lidar.get_info()}")
            return lidar
        except RPLidarException as e:
            print(f"[WARN] get_info a échoué ({e}), reset...")
            lidar.reset()
            time.sleep(2.0)
            lidar.clear_input()
    raise RuntimeError("Le lidar ne répond pas")


def is_inside_valid_zone(wx, wy):
    """Returns True if (wx, wy) is inside the playing area OR inside one of the 3 beacon pockets."""
    if MIN_PLAY_X <= wx <= MAX_PLAY_X and MIN_PLAY_Y <= wy <= MAX_PLAY_Y:
        return True
    for bx, by in KNOWN_BEACONS.values():
        if math.hypot(wx - bx, wy - by) <= BEACON_POCKET_RADIUS_MM:
            return True
    return False


def robot_to_world(rx, ry, rtheta_rad, lx, ly):
    """Transform point (lx, ly) from robot frame to world frame."""
    c, s = math.cos(rtheta_rad), math.sin(rtheta_rad)
    wx = rx + (lx * c - ly * s)
    wy = ry + (lx * s + ly * c)
    return wx, wy


def estimate_pose_2d(matched_pairs):
    """
    Closed-form 2D rigid alignment (Kabsch/Procrustes) from robot-frame points to world-frame points.
    matched_pairs: list of ((lx, ly), (wx, wy)) with len >= 2.
    """
    n = len(matched_pairs)
    mean_lx = sum(p[0][0] for p in matched_pairs) / n
    mean_ly = sum(p[0][1] for p in matched_pairs) / n
    mean_wx = sum(p[1][0] for p in matched_pairs) / n
    mean_wy = sum(p[1][1] for p in matched_pairs) / n

    num_c = 0.0
    num_s = 0.0
    for (lx, ly), (wx, wy) in matched_pairs:
        dlx, dly = lx - mean_lx, ly - mean_ly
        dwx, dwy = wx - mean_wx, wy - mean_wy
        num_c += dlx * dwx + dly * dwy
        num_s += dlx * dwy - dly * dwx

    theta = math.atan2(num_s, num_c)
    c, s = math.cos(theta), math.sin(theta)
    rx = mean_wx - (mean_lx * c - mean_ly * s)
    ry = mean_wy - (mean_lx * s + mean_ly * c)
    return rx, ry, theta


def detect_beacons_and_localize(scan, robot_pose):
    """
    Filters out spectators outside the 2x3m table, detects beacon clusters,
    matches them to the 3 fixed beacons, and updates the robot's (x, y, theta) pose.
    """
    rx, ry, rtheta = robot_pose

    # 1. Convert raw rays to robot frame & reject points outside the table + beacon pockets
    points = []
    for quality, angle_deg, dist in scan:
        if quality <= 0 or not (MIN_DIST_MM <= dist <= MAX_DIST_MM):
            continue

        rel_angle_deg = LIDAR_ANGLE_SIGN * angle_deg + LIDAR_YAW_OFFSET_DEG
        rad = math.radians(rel_angle_deg)
        lx = dist * math.cos(rad)
        ly = dist * math.sin(rad)

        wx, wy = robot_to_world(rx, ry, rtheta, lx, ly)
        if not is_inside_valid_zone(wx, wy):
            continue  # Ignore spectators & walls outside the table

        points.append({
            'rad': rad,
            'dist': dist,
            'lx': lx,
            'ly': ly,
        })

    if not points:
        return robot_pose, {}, []

    points.sort(key=lambda p: p['rad'])

    # 2. Cluster adjacent valid points
    clusters = []
    current_cluster = [points[0]]
    for p in points[1:]:
        prev = current_cluster[-1]
        if math.hypot(p['lx'] - prev['lx'], p['ly'] - prev['ly']) <= MAX_GAP_MM:
            current_cluster.append(p)
        else:
            clusters.append(current_cluster)
            current_cluster = [p]
    clusters.append(current_cluster)

    if len(clusters) > 1:
        if math.hypot(clusters[0][0]['lx'] - clusters[-1][-1]['lx'],
                      clusters[0][0]['ly'] - clusters[-1][-1]['ly']) <= MAX_GAP_MM:
            clusters[0].extend(clusters.pop())

    # 3. Extract cylindrical beacon candidates
    candidates = []
    for cluster in clusters:
        if len(cluster) < 2:
            continue

        front_ref = min(p['dist'] for p in cluster)
        inliers = [p for p in cluster if (p['dist'] - front_ref) <= MAX_RADIAL_SPREAD_MM]
        if len(inliers) < 2:
            continue

        width = math.hypot(inliers[0]['lx'] - inliers[-1]['lx'], inliers[0]['ly'] - inliers[-1]['ly'])
        radial_spread = max(p['dist'] for p in inliers) - front_ref

        if width <= MAX_BEACON_WIDTH_MM and radial_spread <= MAX_RADIAL_SPREAD_MM:
            raw_dist = median(p['dist'] for p in inliers)
            cal_dist = (raw_dist * DIST_SCALE) + DIST_OFFSET_MM + BEACON_RADIUS_MM

            ux = sum(math.cos(p['rad']) for p in inliers)
            uy = sum(math.sin(p['rad']) for p in inliers)
            ray_angle = math.atan2(uy, ux)

            lx = cal_dist * math.cos(ray_angle)
            ly = cal_dist * math.sin(ray_angle)
            wx, wy = robot_to_world(rx, ry, rtheta, lx, ly)

            candidates.append({
                'dist_mm': cal_dist,
                'angle_deg': math.degrees(ray_angle),
                'lx': lx,
                'ly': ly,
                'wx': wx,
                'wy': wy,
                'pts': len(inliers),
            })

    # 4. Associate candidates with the 3 known beacons
    matched_beacons = {}
    matched_pairs = []
    unmatched_in_field = []

    used_names = set()
    for cand in candidates:
        best_name = None
        best_err = BEACON_MATCH_GATE_MM
        for name, (bx, by) in KNOWN_BEACONS.items():
            if name in used_names:
                continue
            err = math.hypot(cand['wx'] - bx, cand['wy'] - by)
            if err < best_err:
                best_err = err
                best_name = name

        if best_name is not None:
            used_names.add(best_name)
            matched_beacons[best_name] = cand
            matched_pairs.append(((cand['lx'], cand['ly']), KNOWN_BEACONS[best_name]))
        else:
            unmatched_in_field.append(cand)

    # 5. Update robot pose if at least 2 beacons are matched
    if len(matched_pairs) >= 2:
        robot_pose = estimate_pose_2d(matched_pairs)

    return robot_pose, matched_beacons, unmatched_in_field


def run_lidar():
    lidar = connect_lidar()
    # Initial pose: (0, 0) at the middle of the yellow zone, facing +X (0 rad)
    robot_pose = (0.0, 0.0, 0.0)

    try:
        lidar.start_motor()
        time.sleep(2.0)

        print("[INFO] Origine (0, 0) = centre de la zone jaune. Filtrage hors-table actif.\n")

        for scan in lidar.iter_scans():
            robot_pose, matched, obstacles = detect_beacons_and_localize(scan, robot_pose)
            rx, ry, rtheta = robot_pose
            heading_deg = math.degrees(rtheta)

            beacons_str = " | ".join(
                f"{name}: {b['dist_mm']:.0f}mm ({b['angle_deg']:+.1f}°)"
                for name, b in matched.items()
            ) or "Aucune"

            print(
                f"POS ROBOT: X={rx:+6.0f}mm  Y={ry:+6.0f}mm  Cap={heading_deg:+5.1f}°  ||  "
                f"Balises ({len(matched)}/3): {beacons_str}"
            )

    except KeyboardInterrupt:
        print("\n[STOP] Interruption utilisateur.")
    except RPLidarException as err:
        print(f"[ERROR RPLidar] {err}")
    finally:
        for fn in (lidar.stop, lidar.stop_motor, lidar.disconnect):
            try:
                fn()
            except Exception:
                pass


if __name__ == '__main__':
    run_lidar()