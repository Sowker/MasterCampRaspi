import time
import threading
import subprocess
import sys
from enum import Enum, auto
import cv2
import numpy as np

# Imports spécifiques au matériel de votre Robot
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

from t11_argument_parser import parse_args
from t11_robot import Robot
from picamera2 import Picamera2
from t6_line_tracking import LinePosition


class Direction:
    FORWARD = "forward"
    BACKWARD = "backward"


# ── PARAMÈTRES DE CONFIGURATION REGLABLES ─────────────────────────────────────
SPEED_MAX_PCT = 48
STEER_CENTER_DEG = 90
MIN_LINE_AREA = 300
CTRL_INTERVAL = 0.05
US_INTERVAL = 0.06  # Intervalle de rafraîchissement ultrason
LED_INTERVAL = 0.1  # Intervalle de rafraîchissement des LED

lock = threading.Lock()
telemetry = {
    "fps": 0.0,
    "line_seen": "NON",
    "error_px": 0,
    "stable_dir": "AUCUNE",
    "distance_mm": 0,
    "speed_pct": 0,
    "emergency": False
}

current_encoded_frame = None
system_running = True


def get_black_mask(roi: np.ndarray) -> np.ndarray:
    """Isole la couleur noir dans la zone d'intérêt."""
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([255, 255, 50])

    mask1 = cv2.inRange(hsv_roi, lower_black, upper_black)
    mask2 = cv2.inRange(hsv_roi, lower_black, upper_black)
    mask_roi = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)


def process_frame(frame: np.ndarray, robot_instance: Robot) -> np.ndarray:
    """Analyse l'image pour trouver une ligne, s'aligner dessus, puis s'arrêter."""
    height, width = frame.shape[:2]
    center_x = width // 2
    output = frame.copy()

    # Définition des zones d'intérêt (ROIs) - On se concentre sur le bas pour l'alignement
    roi_low_top, roi_low_bot = int(height * 0.70), int(height * 0.90)
    mask_low = get_black_mask(frame[roi_low_top:roi_low_bot, 0:width])
    M_low = cv2.moments(mask_low)

    # Affichage des lignes de guidage
    cv2.line(output, (0, roi_low_top), (width, roi_low_top), (0, 140, 255), 1)
    cv2.line(output, (center_x, 0), (center_x, height), (255, 0, 0), 1)

    line_seen = "NON"
    final_angle_delta = 0.0
    calculated_speed = 0
    stable_dir = "AUCUNE"
    border_color = (0, 255, 0)

    # ── LOGIQUE D'ALIGNEMENT ET TRANSITION ──
    if M_low["m00"] > MIN_LINE_AREA:
        line_seen = "OUI"

        # Position X du centre de la ligne noire
        cx_low = int(M_low["m10"] / M_low["m00"])
        cy_low = roi_low_top + int(M_low["m01"] / M_low["m00"])
        cv2.circle(output, (cx_low, cy_low), 6, (0, 255, 0), -1)

        # Calcul de l'erreur par rapport au centre de l'image (en pixels)
        error_px = cx_low - center_x

        # On braque proportionnellement à l'erreur (MAX_STEER_DELTA = 45)
        final_angle_delta = (error_px / center_x) * 45

        # Si l'erreur est très petite (<= 25px), la ligne est au milieu : on s'arrête !
        if abs(error_px) <= 25:
            final_angle_delta = 0.0
            calculated_speed = 0
            stable_dir = "CENTRÉ SUR LA LIGNE - ARRÊT"
            border_color = (0, 0, 255)  # Rouge (Arrêt)

            # Transition vers l'état suivant
            with robot_instance.state.lock:
                robot_instance.state.action = "Line following"
        else:
            # La ligne est visible mais pas centrée : on tourne en roulant doucement
            calculated_speed = 37  # SPEED_MIN_PCT
            stable_dir = f"AJUSTEMENT ({error_px}px)"
            border_color = (255, 165, 0)  # Orange (Ajustement)
    else:
        # Aucune ligne en vue, on roule tout droit pour la chercher.
        final_angle_delta = 0.0
        calculated_speed = SPEED_MAX_PCT
        stable_dir = "RECHERCHE LIGNE..."
        border_color = (0, 255, 0)  # Vert (Recherche)

    # Récupération sécurisée de l'état ultrason / arrêt d'urgence avant envoi moteur
    with robot_instance.state.lock:
        is_emergency = robot_instance.state.emergency_stop
        current_dist = getattr(robot_instance.state, 'distance_mm', 0)

        if is_emergency:
            calculated_speed = 0

        robot_instance.state.calculated_speed = calculated_speed
        robot_instance.state.calculated_angle = int(STEER_CENTER_DEG + final_angle_delta)

    # Incrustation vidéo
    cv2.putText(output, f"Servo Delta: {int(final_angle_delta)}deg | Vitesse: {calculated_speed}%", (10, height - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(output, f"STRAT: {stable_dir}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)

    with lock:
        telemetry["line_seen"] = line_seen
        telemetry["error_px"] = int(final_angle_delta)
        telemetry["stable_dir"] = stable_dir
        telemetry["distance_mm"] = current_dist
        telemetry["speed_pct"] = calculated_speed
        telemetry["emergency"] = is_emergency

    return output


# THREADS MATÉRIELS ET SENSEURS ASYNCHRONES
def thread_controller_camera_line(robot: Robot, interval: float) -> None:
    """Boucle des mouvements du robot"""
    stop = False
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency = robot.state.emergency_stop
            # Utilisation de getattr pour éviter les crashs si importé par un autre fichier
            target_speed = getattr(robot.state, 'calculated_speed', 0)
            target_angle = getattr(robot.state, 'calculated_angle', STEER_CENTER_DEG)
            post_time = getattr(robot.state, 'post_time', 0)

        if emergency:
            robot.motor.stop()
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)
            time.sleep(interval)
            continue

        if target_speed > 0:
            robot.head.set_angle_motor(0, 180 - target_angle)
            robot.motor.drive(Direction.FORWARD, target_speed, fast_accel=True)
            stop = False  # RAZ du flag d'arrêt si on se remet à rouler
        else:
            if stop:
                if time.time() <= post_time + 4:
                    robot.head.set_angle_motor(0, 180 - target_angle)
                else:
                    robot.motor.stop()
                    robot.head.set_angle_motor(0, STEER_CENTER_DEG)
                    with robot.state.lock:
                        if not robot.state.running:
                            break
                        robot.state.action = "Line following"

            else:
                stop = True
                with robot.state.lock:
                    robot.state.post_time = time.time()

        time.sleep(interval)

    robot.motor.stop()
    robot.head.set_angle_motor(0, STEER_CENTER_DEG)


def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Mesure continue de la distance avant et levée du drapeau d'urgence."""
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break

        try:
            dist_mm = robot.ultrasonic.read_mm()
        except Exception:
            dist_mm = 999  # Fallback si erreur matérielle d'écho

        with robot.state.lock:
            robot.state.distance_mm = dist_mm
            robot.state.emergency_stop = dist_mm < 120

        time.sleep(interval)


def thread_LED(robot: Robot, interval: float):
    """Régulation dynamique de la signalisation lumineuse selon la cinématique du robot."""
    last_front_state = None

    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency = robot.state.emergency_stop
            # Utilisation de getattr pour éviter les crashs si importé par un autre fichier
            angle = getattr(robot.state, 'calculated_angle', STEER_CENTER_DEG)

        if emergency:
            target_state = 'warning'
            robot.led.warning()
        elif angle < (STEER_CENTER_DEG - 10):
            target_state = 'left'
            robot.led.clignotant_gauche()
        elif angle > (STEER_CENTER_DEG + 10):
            target_state = 'right'
            robot.led.clignotant_droit()
        else:
            target_state = None
            robot.led.arreter_clignotants()
            robot.led.arreter_warning()

        if target_state != last_front_state:
            try:
                robot.front_leds.set_blink(target_state)
            except Exception:
                pass
            last_front_state = target_state

        time.sleep(interval)

    try:
        robot.front_leds.cancel_blink()
    except Exception:
        pass


def thread_camera_loop(robot_instance: Robot, external_camera=None):
    """Boucle autonome qui lit la caméra en continu et met à jour les données"""
    global system_running, current_encoded_frame

    # Accepte une caméra externe (venant du Main) pour éviter le crash "Device Busy"
    if external_camera is None:
        picam = Picamera2()
        config = picam.create_video_configuration(main={"size": (640, 480)})
        picam.configure(config)
        picam.start()
        time.sleep(0.1)
        owns_camera = True
    else:
        picam = external_camera
        owns_camera = False

    frame_count = 0
    t0 = time.time()

    try:
        while system_running:
            frame = picam.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Calcul du FPS
            frame_count += 1
            elapsed = time.time() - t0
            if elapsed >= 1.0:
                with lock:
                    telemetry["fps"] = round(frame_count / elapsed, 1)
                frame_count, t0 = 0, time.time()

            # Analyse l'image et met à jour les consignes moteurs
            processed = process_frame(frame, robot_instance)

            # Encode l'image traitée en JPG et la stocke
            _, enc = cv2.imencode(".jpg", processed)
            with lock:
                current_encoded_frame = enc.tobytes()

            time.sleep(0.01)
    except Exception as e:
        print(f"Erreur flux vidéo autonome: {e}")
    finally:
        if owns_camera:
            picam.stop()
            picam.close()


# POINT D'ENTRÉE PRINCIPAL D'EXÉCUTION (POUR TESTER CE FICHIER SEUL)
if __name__ == "__main__":
    args = parse_args()

    subprocess.run(["sudo", "pkill", "-f", "rpicam"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)

    robot = Robot(args)
    robot.init()

    # Orientation physique initiale de l'axe vertical caméra
    robot.head.set_angle_motor(2, 60)

    # Initialisation explicite des variables d'état pour éviter les crashs locaux
    with robot.state.lock:
        robot.state.calculated_speed = 0
        robot.state.calculated_angle = STEER_CENTER_DEG
        robot.state.distance_mm = 999
        robot.state.post_time = 0
        robot.state.emergency_stop = False

    global_robot_ref = robot

    # Démarrage synchrone de tous les threads utiles
    threads = [
        threading.Thread(target=thread_controller_camera_line, args=(robot, CTRL_INTERVAL), name="CTRL", daemon=True),
        threading.Thread(target=thread_ultrasonic, args=(robot, US_INTERVAL), name="US", daemon=True),
        threading.Thread(target=thread_LED, args=(robot, LED_INTERVAL), name="LED", daemon=True),
        threading.Thread(target=thread_camera_loop, args=(robot,), name="CAM_AUTO", daemon=True),
    ]

    for t in threads:
        t.start()

    try:
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        pass

    finally:
        system_running = False

        with robot.state.lock:
            robot.state.running = False

        for t in threads:
            t.join(timeout=1.0)

        robot.shutdown()