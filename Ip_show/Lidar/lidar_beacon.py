import time
import math
import logging
from statistics import median
from rplidar import RPLidar, RPLidarException

logging.basicConfig(level=logging.WARNING)

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

# --- CALIBRATION & EUROBOT BEACON PARAMETERS ---
BEACON_RADIUS_MM = 40.0     # 40 mm for an 80 mm diameter Eurobot beacon (0.0 for front-surface distance)
DIST_SCALE = 1.0            # Multiplicative gain calibration (adjust if error scales with distance)
DIST_OFFSET_MM = -210.0        # Constant offset in mm (e.g., -250.0 if retroreflective tape biases +250 mm)

MIN_DIST_MM = 100
MAX_DIST_MM = 3000
MAX_GAP_MM = 65             # ~2.3x ray spacing at 1.6 m (prevents merging background points)
MAX_BEACON_WIDTH_MM = 130   # Eurobot beacons are 80 mm wide; allows margin for ray discretization
MAX_RADIAL_SPREAD_MM = 80   # Rejects slanted walls/borders whose depth varies too much across the cluster


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


def detect_beacons(
    scan,
    min_dist_mm=MIN_DIST_MM,
    max_dist_mm=MAX_DIST_MM,
    max_gap_mm=MAX_GAP_MM,
    max_beacon_width_mm=MAX_BEACON_WIDTH_MM,
    max_radial_spread_mm=MAX_RADIAL_SPREAD_MM,
):
    """Detects isolated cylindrical beacons in a 360-degree scan."""
    # 1. Filter valid points within range window
    points = []
    for quality, angle, dist in scan:
        if quality > 0 and min_dist_mm <= dist <= max_dist_mm:
            rad = math.radians(angle)
            points.append({
                'angle': angle,
                'rad': rad,
                'dist': dist,
                'x': dist * math.cos(rad),
                'y': dist * math.sin(rad),
            })

    if not points:
        return []

    points.sort(key=lambda p: p['angle'])

    # 2. Cluster adjacent points by 2D Euclidean distance
    clusters = []
    current_cluster = [points[0]]

    for p in points[1:]:
        prev = current_cluster[-1]
        if math.hypot(p['x'] - prev['x'], p['y'] - prev['y']) <= max_gap_mm:
            current_cluster.append(p)
        else:
            clusters.append(current_cluster)
            current_cluster = [p]
    clusters.append(current_cluster)

    # Merge wrap-around cluster across 360° -> 0° boundary
    if len(clusters) > 1:
        first_p = clusters[0][0]
        last_p = clusters[-1][-1]
        if math.hypot(first_p['x'] - last_p['x'], first_p['y'] - last_p['y']) <= max_gap_mm:
            clusters[0].extend(clusters.pop())

    # 3. Filter clusters by width & depth spread, then compute robust median range
    beacons = []
    for cluster in clusters:
        if len(cluster) < 2:
            continue

        # Trim grazing edge-bleed points that jump far behind the cluster's front surface
        front_ref = min(p['dist'] for p in cluster)
        inliers = [p for p in cluster if (p['dist'] - front_ref) <= max_radial_spread_mm]
        if len(inliers) < 2:
            continue

        width = math.hypot(inliers[0]['x'] - inliers[-1]['x'], inliers[0]['y'] - inliers[-1]['y'])
        radial_spread = max(p['dist'] for p in inliers) - front_ref

        if width <= max_beacon_width_mm and radial_spread <= max_radial_spread_mm:
            raw_dist_mm = median(p['dist'] for p in inliers)
            calibrated_dist_mm = (raw_dist_mm * DIST_SCALE) + DIST_OFFSET_MM + BEACON_RADIUS_MM

            # Vector mean of unit rays avoids radial-outlier skew and 360° wrap issues
            ux = sum(math.cos(p['rad']) for p in inliers)
            uy = sum(math.sin(p['rad']) for p in inliers)
            center_angle = math.degrees(math.atan2(uy, ux)) % 360

            beacons.append({
                'distance_mm': calibrated_dist_mm,
                'raw_dist_mm': raw_dist_mm,
                'angle_deg': center_angle,
                'points_count': len(inliers),
                'width_mm': width,
            })

    beacons.sort(key=lambda b: b['distance_mm'])
    return beacons


def run_lidar():
    lidar = connect_lidar()
    try:
        lidar.start_motor()
        time.sleep(2.0)

        print(f"{'Statut':<15} | {'Calibré':<10} | {'Brut':<10} | {'Angle':<8} | {'Largeur':<9} | Pts")
        print("-" * 72)

        for scan in lidar.iter_scans():
            beacons = detect_beacons(scan)

            if beacons:
                # Show up to the 3 closest detected beacons to verify no wall is masking the target
                summary = "  ||  ".join(
                    f"{b['distance_mm']:.0f}mm (raw {b['raw_dist_mm']:.0f}, {b['angle_deg']:.1f}°, w={b['width_mm']:.0f}mm, {b['points_count']}p)"
                    for b in beacons[:3]
                )
                print(f"{'BALISE(S)':<15} | {summary}")
            else:
                print(f"{'Aucune balise':<15} | {'---':<10} | {'---':<10} | {'---':<8} | {'---':<9} | {len(scan)} pts")

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