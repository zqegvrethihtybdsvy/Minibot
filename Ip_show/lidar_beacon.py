import time
import math
import logging
from statistics import median
from rplidar import RPLidar, RPLidarException

logging.basicConfig(level=logging.WARNING)

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

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

def detect_beacons(scan, min_dist_mm=50, max_dist_mm=2000, max_gap_mm=150, max_beacon_width_mm=250):
    """
    Detects small isolated objects (beacons) in a 360-degree scan.
    
    :param scan: Raw scan list [(quality, angle, dist), ...]
    :param min_dist_mm: Minimum detection distance (0.1m)
    :param max_dist_mm: Maximum detection distance (3.0m)
    :param max_gap_mm: Maximum distance between 2 points to be in the same cluster
    :param max_beacon_width_mm: Maximum physical diameter of the target beacon
    :return: List of dicts [{'distance_mm': ..., 'angle_deg': ..., 'points': ...}]
    """
    # 1. Filter valid points within target range window
    points = []
    for quality, angle, dist in scan:
        if quality > 0 and min_dist_mm <= dist <= max_dist_mm:
            rad = math.radians(angle)
            x = dist * math.cos(rad)
            y = dist * math.sin(rad)
            points.append({'angle': angle, 'dist': dist, 'x': x, 'y': y})

    if not points:
        return []

    # Sort points chronologically by angle
    points.sort(key=lambda p: p['angle'])

    # 2. Cluster adjacent points based on Euclidean distance
    clusters = []
    current_cluster = [points[0]]

    for p in points[1:]:
        prev = current_cluster[-1]
        dist_between = math.hypot(p['x'] - prev['x'], p['y'] - prev['y'])

        if dist_between <= max_gap_mm:
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

    # 3. Filter clusters by size and compute geometric centroid
    beacons = []
    for cluster in clusters:
        # Ignore single-point noise reflection
        if len(cluster) < 2:
            continue

        # Estimate physical width of the cluster
        width = math.hypot(cluster[0]['x'] - cluster[-1]['x'], cluster[0]['y'] - cluster[-1]['y'])

        # Keep cluster if it matches beacon size constraints
        if width <= max_beacon_width_mm:
            avg_x = sum(p['x'] for p in cluster) / len(cluster)
            avg_y = sum(p['y'] for p in cluster) / len(cluster)

            center_dist = math.hypot(avg_x, avg_y)
            center_angle = (math.degrees(math.atan2(avg_y, avg_x))) % 360

            beacons.append({
                'distance_mm': center_dist,
                'angle_deg': center_angle,
                'points_count': len(cluster),
                'width_mm': width
            })

    # Sort closest beacon first
    beacons.sort(key=lambda b: b['distance_mm'])
    return beacons

def run_lidar():
    lidar = connect_lidar()
    try:
        lidar.start_motor()
        time.sleep(2.0)

        print(f"{'Statut':<15} | {'Distance':<12} | {'Angle':<10} | Nb Points")
        print("-" * 55)

        for scan in lidar.iter_scans():
            beacons = detect_beacons(scan, min_dist_mm=100, max_dist_mm=3000)

            if beacons:
                primary = beacons[0]  # Closest detected beacon
                print(f"{'BALISE DÉTECTÉE':<15} | {primary['distance_mm']:<6.0f} mm    | {primary['angle_deg']:<6.1f}°    | {primary['points_count']} pts")
            else:
                print(f"{'Aucune balise':<15} | {'---':<12} | {'---':<10} | {len(scan)} pts")

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