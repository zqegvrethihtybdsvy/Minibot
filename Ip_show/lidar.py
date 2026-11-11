import time
import logging
from statistics import median
from rplidar import RPLidar, RPLidarException

logging.basicConfig(level=logging.WARNING)  # affiche les warnings de la lib

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200

def connect_lidar(retries=3):
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=3)
    lidar.stop()          # coupe un scan éventuellement resté actif
    lidar.clean_input()   # vide le buffer, sans rouvrir le port
    for _ in range(retries):
        try:
            print(f"[LiDAR] Info : {lidar.get_info()}")
            return lidar
        except RPLidarException as e:
            print(f"[WARN] get_info a échoué ({e}), reset...")
            lidar.reset()
            time.sleep(2.0)
            lidar.clean_input()
    raise RuntimeError("Le lidar ne répond pas")

def zone_distance(scan, lo, hi):
    """Médiane des distances (mm) entre lo et hi degrés, ou None."""
    d = []
    for quality, angle, dist in scan:
        if dist == 0 or quality == 0:
            continue
        in_zone = (angle >= lo or angle <= hi) if lo > hi else (lo <= angle <= hi)
        if in_zone:
            d.append(dist)
    return median(d) if d else None

def run_lidar():
    lidar = connect_lidar()
    try:
        lidar.start_motor()
        time.sleep(2.0)   # laisse le moteur atteindre sa vitesse

        print(f"{'Avant':<12} | {'Droite':<12} | Nb points")
        for scan in lidar.iter_scans():   # pas de max_buf_meas=500
            front = zone_distance(scan, 350, 10)
            right = zone_distance(scan, 80, 100)
            f = f"{front:.0f} mm" if front else "---"
            r = f"{right:.0f} mm" if right else "---"
            print(f"{f:<12} | {r:<12} | {len(scan)}")
    except KeyboardInterrupt:
        print("\n[STOP]")
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