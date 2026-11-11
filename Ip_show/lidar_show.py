import time
import sys
import serial
from rplidar import RPLidar, RPLidarException

PORT_NAME = '/dev/ttyUSB0'
BAUDRATE = 115200  # Changer à 256000 si vous avez un modèle A2M8 / A3

def hard_reset_serial_port(port, baudrate):
    """
    Simule une réinitialisation de bus : ouvre le port en brut, 
    purge les buffers d'entrée/sortie et envoie la commande STOP binaire (0x25).
    """
    print(f"[RESET] Nettoyage bas niveau du port {port}...")
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        # 1. Envoi de la commande de STOP binaire explicite du RPLiDAR
        ser.write(b'\xA5\x25')
        time.sleep(0.1)
        # 2. Vider totalement les buffers FIFO
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.close()
        time.sleep(0.5)
        print("[RESET] Port série purgé et prêt.")
    except Exception as e:
        print(f"[WARN] Impossible de réinitialiser le port série : {e}")

def run_lidar():
    # Étape 1 : Flush préalable du port
    hard_reset_serial_port(PORT_NAME, BAUDRATE)

    # Étape 2 : Initialisation RPLidar
    print(f"[LiDAR] Connexion au RPLiDAR sur {PORT_NAME}...")
    lidar = RPLidar(PORT_NAME, baudrate=BAUDRATE, timeout=3)

    try:
        # Relancer le moteur
        print("[LiDAR] Démarrage du moteur...")
        lidar.start_motor()
        time.sleep(1.0)

        # Récupération des infos avec gestion du retry
        try:
            info = lidar.get_info()
            print(f"[LiDAR] Device Info : {info}")
        except RPLidarException as e:
            print(f"[WARN] Échec get_info ({e}), tentative de réinitialisation logicielle...")
            lidar.reset()
            time.sleep(1.0)

        print("\n=== Démarrage de la lecture du balayage (Ctrl+C pour quitter) ===")
        print(f"{'Angle 0° (Avt)':<15} | {'Angle 90° (Drte)':<15} | {'Nb Points':<10}")
        print("-" * 50)

        for scan in lidar.iter_scans(max_buf_meas=500):
            front_dist = "---"
            right_dist = "---"
            
            for quality, angle, distance in scan:
                if distance == 0:
                    continue
                # Mesure dans un cône de +/- 10°
                if angle >= 350 or angle <= 10:
                    front_dist = f"{distance:.0f} mm"
                elif 80 <= angle <= 100:
                    right_dist = f"{distance:.0f} mm"

            print(f"{front_dist:<15} | {right_dist:<15} | {len(scan):<10} pts")

    except KeyboardInterrupt:
        print("\n[STOP] Interruption utilisateur.")
    except RPLidarException as err:
        print(f"\n[ERROR RPLidar] {err}")
    finally:
        print("[LiDAR] Fermeture propre de la connexion...")
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except Exception:
            pass
        print("[LiDAR] Arrêté.")

if __name__ == '__main__':
    run_lidar()