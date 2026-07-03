import time
import threading
import subprocess
import sys
from enum import Enum, auto
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# Imports matériels
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

from t11_argument_parser import parse_args
from t11_robot import Robot
from picamera2 import Picamera2


class Direction:
    FORWARD = "forward"
    BACKWARD = "backward"


# ── PARAMÈTRES DE CONFIGURATION RÉGLABLES ─────────────────────────────────────
SPEED_MAX_PCT = 48
SPEED_MIN_PCT = 37

STEER_CENTER_DEG = 90
MAX_STEER_DELTA = 45  # Braquage max autorisé (90 +/- 45)

MIN_LINE_AREA = 300
CTRL_INTERVAL = 0.05
US_INTERVAL = 0.06
LED_INTERVAL = 0.1

THRESHOLD_DEADZONE = 5
ALPHA_SMOOTHING = 0.35
THRESHOLD_URGENCY = 30

# Pondérations nominales
WEIGHT_POSITION = 0.65
WEIGHT_DIRECTION = 0.35

# Variables de lissage global
smoothed_angle_delta = 0.0

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

# ── NOUVELLES VARIABLES LOCALES POUR ÉVITER LES ERREURS DE STATE ──────────────
local_control = {
    "speed": 0,
    "angle": STEER_CENTER_DEG
}

current_encoded_frame = None
system_running = True

app = Flask(__name__)
global_robot_ref = None
global_camera_ref = None


def get_red_mask(roi: np.ndarray) -> np.ndarray:
    """Isole la couleur rouge dans la zone d'intérêt."""
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
    mask_roi = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    return cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)


def process_frame(frame: np.ndarray, robot_instance: Robot) -> np.ndarray:
    """Analyse les bandes à longue distance et gère la trajectoire."""
    global smoothed_angle_delta
    height, width = frame.shape[:2]
    center_x = width // 2
    output = frame.copy()

    roi_low_top, roi_low_bot = int(height * 0.70), int(height * 0.90)
    roi_high_top, roi_high_bot = int(height * 0.20), int(height * 0.40)

    mask_low = get_red_mask(frame[roi_low_top:roi_low_bot, 0:width])
    mask_high = get_red_mask(frame[roi_high_top:roi_high_bot, 0:width])

    M_low = cv2.moments(mask_low)
    M_high = cv2.moments(mask_high)

    cv2.line(output, (0, roi_low_top), (width, roi_low_top), (0, 140, 255), 1)
    cv2.line(output, (0, roi_high_top), (width, roi_high_top), (0, 255, 255), 1)
    cv2.line(output, (center_x, 0), (center_x, height), (255, 0, 0), 1)

    pt_low = None
    pt_high = None

    if M_low["m00"] > MIN_LINE_AREA:
        cx_low = int(M_low["m10"] / M_low["m00"])
        cy_low = roi_low_top + int(M_low["m01"] / M_low["m00"])
        pt_low = (cx_low, cy_low)
        cv2.circle(output, pt_low, 6, (0, 255, 0), -1)

    if M_high["m00"] > MIN_LINE_AREA:
        cx_high = int(M_high["m10"] / M_high["m00"])
        cy_high = roi_high_top + int(M_high["m01"] / M_high["m00"])
        pt_high = (cx_high, cy_high)
        cv2.circle(output, pt_high, 6, (0, 255, 255), -1)

    target_angle_delta = 0.0
    line_seen = "NON"
    stable_dir = "RECHERCHE LIGNE"
    border_color = (0, 255, 0)

    force_low_speed = False
    bypass_smoothing = False

    # ── CALCUL TRAJECTOIRE ──
    if pt_low is not None:
        line_seen = "OUI"
        error_low_px = pt_low[0] - center_x
        angle_base_low = (error_low_px / center_x) * MAX_STEER_DELTA

        if abs(error_low_px) > THRESHOLD_URGENCY:
            target_angle_delta = angle_base_low
            stable_dir = f"URGENCE CRITIQUE BAS"
            border_color = (0, 100, 255)
            bypass_smoothing = True

        elif pt_high is not None:
            cv2.line(output, pt_low, pt_high, (255, 0, 255), 2)
            dx = pt_high[0] - pt_low[0]
            dy = pt_low[1] - pt_high[1]
            angle_vector_deg = np.degrees(np.arctan2(dx, dy))

            if abs(angle_vector_deg) > 20.0:
                direction_sign = np.sign(dx) if dx != 0 else np.sign(error_low_px)
                target_angle_delta = direction_sign * MAX_STEER_DELTA
                stable_dir = f"🚨 COUPE-FILE ANTICIPÉ ({int(angle_vector_deg)}°)"
                border_color = (255, 0, 128)
                force_low_speed = True
                bypass_smoothing = True
            else:
                midpoint_x = (pt_low[0] + pt_high[0]) / 2.0
                error_position_px = midpoint_x - center_x
                angle_from_position = (error_position_px / center_x) * MAX_STEER_DELTA
                angle_from_direction = (angle_vector_deg / 45.0) * MAX_STEER_DELTA

                if abs(error_low_px) > 15 and abs(dx) < 12:
                    target_angle_delta = (angle_from_position * 0.85) + (angle_from_direction * 0.15)
                    stable_dir = "COMBO VERTICALE"
                    border_color = (255, 191, 0)
                else:
                    target_angle_delta = (angle_from_position * WEIGHT_POSITION) + (
                                angle_from_direction * WEIGHT_DIRECTION)
                    stable_dir = "COMBO DIRECT + POS"
        else:
            target_angle_delta = angle_base_low * 1.3
            stable_dir = "SUIVI SIMPLE BAS"
            bypass_smoothing = True

    elif pt_high is not None:
        line_seen = "OUI"
        error_high_px = pt_high[0] - center_x
        target_angle_delta = (error_high_px / center_x) * MAX_STEER_DELTA * 1.2
        stable_dir = "ACCROCHE SECU HAUT"

    # ── FILTRAGE ET CONSIGNES ──
    if line_seen == "OUI":
        if bypass_smoothing:
            smoothed_angle_delta = target_angle_delta
        else:
            smoothed_angle_delta = (ALPHA_SMOOTHING * target_angle_delta) + (
                        (1.0 - ALPHA_SMOOTHING) * smoothed_angle_delta)

        if abs(smoothed_angle_delta) <= THRESHOLD_DEADZONE:
            final_angle_delta = 0.0
        else:
            final_angle_delta = np.clip(smoothed_angle_delta, -MAX_STEER_DELTA, MAX_STEER_DELTA)

        if force_low_speed:
            calculated_speed = SPEED_MIN_PCT
        else:
            turn_ratio = abs(final_angle_delta) / MAX_STEER_DELTA
            calculated_speed = int(SPEED_MAX_PCT - (turn_ratio * (SPEED_MAX_PCT - SPEED_MIN_PCT)))
    else:
        final_angle_delta = 0.0
        calculated_speed = 0
        stable_dir = "LIGNE PERDUE"
        border_color = (0, 0, 255)

    # Récupération de l'état ultrason global de manière standard
    with robot_instance.state.lock:
        is_emergency = robot_instance.state.emergency_stop
        current_dist = getattr(robot_instance.state, 'distance_mm', 0)

    if is_emergency:
        calculated_speed = 0

    # Stockage dans le dictionnaire de contrôle LOCAL
    with lock:
        local_control["speed"] = calculated_speed
        local_control["angle"] = int(STEER_CENTER_DEG + final_angle_delta)

        telemetry["line_seen"] = line_seen
        telemetry["error_px"] = int(final_angle_delta)
        telemetry["stable_dir"] = stable_dir
        telemetry["distance_mm"] = current_dist
        telemetry["speed_pct"] = calculated_speed
        telemetry["emergency"] = is_emergency

    cv2.putText(output, f"Servo Delta: {int(final_angle_delta)}deg | Vitesse: {calculated_speed}%", (10, height - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    cv2.putText(output, f"STRAT: {stable_dir}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, border_color, 2)

    return output


def thread_controller_camera_line(robot: Robot, interval: float) -> None:
    """Boucle matérielle principale. Lit local_control au lieu de robot.state."""
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency = robot.state.emergency_stop

        # Lecture sécurisée des consignes calculées par la vision
        with lock:
            target_speed = local_control["speed"]
            target_angle = local_control["angle"]

        if emergency:
            robot.motor.stop()
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)
            time.sleep(interval)
            continue

        if target_speed > 0:
            robot.head.set_angle_motor(0, 180 - target_angle)
            robot.motor.drive(Direction.FORWARD, target_speed, fast_accel=True)
        else:
            robot.motor.stop()
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)

        time.sleep(interval)

    robot.motor.stop()
    robot.head.set_angle_motor(0, STEER_CENTER_DEG)


def thread_ultrasonic(robot: Robot, interval: float) -> None:
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
        try:
            dist_mm = robot.ultrasonic.read_mm()
        except Exception:
            dist_mm = 999
        with robot.state.lock:
            robot.state.distance_mm = dist_mm
            robot.state.emergency_stop = dist_mm < 120
        time.sleep(interval)


def thread_LED(robot: Robot, interval: float):
    last_front_state = None
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency = robot.state.emergency_stop

        with lock:
            angle = local_control["angle"]

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


def thread_camera_loop(robot_instance: Robot, camera_instance=None):
    global system_running, current_encoded_frame
    frame_count = 0
    t0 = time.time()

    time.sleep(0.2)

    try:
        while system_running:
            if camera_instance is not None:
                frame = camera_instance.capture_array()
            elif global_camera_ref is not None:
                frame = global_camera_ref.capture_array()
            elif hasattr(robot_instance, 'camera') and robot_instance.camera is not None:
                frame = robot_instance.camera.capture_array()
            else:
                time.sleep(0.05)
                continue

            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            frame_count += 1
            elapsed = time.time() - t0
            if elapsed >= 1.0:
                with lock:
                    telemetry["fps"] = round(frame_count / elapsed, 1)
                frame_count, t0 = 0, time.time()

            processed = process_frame(frame, robot_instance)
            _, enc = cv2.imencode(".jpg", processed)
            with lock:
                current_encoded_frame = enc.tobytes()

            time.sleep(0.02)
    except Exception as e:
        print(f"Erreur flux vidéo autonome camera_line3: {e}")


def generate_frames(robot_instance: Robot):
    global system_running, current_encoded_frame
    while system_running:
        if current_encoded_frame is not None:
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + current_encoded_frame + b"\r\n")
        time.sleep(0.05)


HTML_INTERFACE = """..."""


@app.route("/")
def index():
    return render_template_string(HTML_INTERFACE)


@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(global_robot_ref), mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/data")
def get_data():
    with lock:
        return jsonify(telemetry)


if __name__ == "__main__":
    args = parse_args()
    subprocess.run(["sudo", "pkill", "-f", "rpicam"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)

    robot = Robot(args)
    robot.init()

    robot.head.set_angle_motor(2, 60)

    picam = Picamera2()
    picam.configure(picam.create_video_configuration(main={"size": (640, 480)}))
    picam.start()
    global_camera_ref = picam
    global_robot_ref = robot

    threads = [
        threading.Thread(target=thread_controller_camera_line, args=(robot, CTRL_INTERVAL), name="CTRL", daemon=True),
        threading.Thread(target=thread_ultrasonic, args=(robot, US_INTERVAL), name="US", daemon=True),
        threading.Thread(target=thread_LED, args=(robot, LED_INTERVAL), name="LED", daemon=True),
        threading.Thread(target=thread_camera_loop, args=(robot, picam), name="CAM_AUTO", daemon=True),
        threading.Thread(
            target=lambda: app.run(host="0.0.0.0", port=5002, debug=False, threaded=True, use_reloader=False),
            name="WEB", daemon=True)
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
        picam.stop()
        picam.close()
        robot.shutdown()