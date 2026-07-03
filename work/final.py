import threading
import time
import sys
import cv2
import numpy as np
from typing import Dict, List, Callable
from flask import Flask, Response, render_template_string

# Configuration matérielle
from picamera2 import Picamera2
from logger import get_logger
from t11_argument_parser import parse_args
from t11_robot import Robot

# Threads — Suivi de Ligne Via Capteurs IR
from t11_threads import (
    thread_ultrasonic as t11_thread_ultrasonic,
    thread_line as t11_thread_line,
    thread_LED as t11_thread_LED,
    thread_controller as t11_thread_controller,
    thread_buzzer as t11_thread_buzzer
)

# Threads — Labyrinthe
from labyrinthe_threads import (
    thread_ultrasonic as labyrinthe_thread_ultrasonic,
    thread_drive as labyrinthe_thread_drive
)

# Threads — Suivi de Ligne Via Caméra Autonome
from camera_line3 import (
    thread_controller_camera_line as thread_camera_line,
    CTRL_INTERVAL,
    thread_ultrasonic as thread_camera_line_US,
    US_INTERVAL,
    thread_LED as thread_camera_line_LED,
    LED_INTERVAL,
    thread_camera_loop,
    app as app_camera_line
)

frame_lock = threading.Lock()
latest_frame = None
system_running = True
target_step = "Line following"

# Configuration du serveur Flask global pour la supervision (Port 5001)
app_global = Flask(__name__)


class StepConfig:
    """Structure de données pour configurer chaque étape du robot."""

    def __init__(self, camera_angle: int, thread_factory: Callable[[], List[threading.Thread]]):
        self.camera_angle = camera_angle
        self.thread_factory = thread_factory
        self.active_threads: List[threading.Thread] = []

    def start(self, robot: Robot) -> None:
        """oriente la caméra, génère les threads et les lance."""
        robot.head.set_angle_motor(2, self.camera_angle)
        self.active_threads = self.thread_factory()
        for thread in self.active_threads:
            thread.start()

    def stop(self) -> None:
        """Attend la fin des threads de cette étape."""
        for thread in self.active_threads:
            if thread.is_alive():
                thread.join(timeout=0.5)
        self.active_threads.clear()


class RobotStepManager:
    """Gère les transitions d'états du robot"""

    def __init__(self, robot: Robot, camera_instance: Picamera2, args):
        self.robot = robot
        self.camera = camera_instance
        self.args = args
        self.current_step: str = "Line following"

        # Mapping pour la console clavier
        self.step_mapping = {
            "1": "Line following",
            "2": "Labyrinthe",
            "3": "Camera Line",
            "4": "Obstacles"
        }

        # Définition des stratégies de chaque étape
        self.steps: Dict[str, StepConfig] = {
            "Line following": StepConfig(
                camera_angle=90,
                thread_factory=lambda: [
                    threading.Thread(target=t11_thread_ultrasonic, args=(robot, args.sensor_interval), name="US_IR",
                                     daemon=True),
                    threading.Thread(target=t11_thread_line, args=(robot, args.sensor_interval), name="LINE_IR",
                                     daemon=True),
                    threading.Thread(target=t11_thread_LED, args=(robot, args.sensor_interval), name="LED_IR",
                                     daemon=True),
                    threading.Thread(target=t11_thread_controller, args=(robot, args.ctrl_interval), name="CTRL_IR",
                                     daemon=True),
                    threading.Thread(target=t11_thread_buzzer, args=(robot,), name="BUZZER", daemon=True),
                ]
            ),
            "Labyrinthe": StepConfig(
                camera_angle=110,
                thread_factory=lambda: [
                    threading.Thread(target=labyrinthe_thread_ultrasonic, args=(robot, args.sensor_interval),
                                     name="US_Labyrinthe", daemon=True),
                    threading.Thread(target=labyrinthe_thread_drive, args=(robot, args.sensor_interval, self.camera),
                                     name="Camera_Labyrinthe", daemon=True)
                ]
            ),
            "Camera Line": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=thread_camera_line, args=(robot, CTRL_INTERVAL), name="CTRL", daemon=True),
                    threading.Thread(target=thread_camera_line_US, args=(robot, US_INTERVAL), name="US", daemon=True),
                    threading.Thread(target=thread_camera_line_LED, args=(robot, LED_INTERVAL), name="LED",
                                     daemon=True),
                    threading.Thread(target=thread_camera_loop, args=(robot,), name="CAM_AUTO", daemon=True),
                    threading.Thread(
                        target=lambda: app_camera_line.run(host="0.0.0.0", port=5000, debug=False, threaded=True,
                                                           use_reloader=False),
                        name="WEB_CAM_LINE", daemon=True
                    )
                ]
            ),
            "Obstacles": StepConfig(
                camera_angle=90,
                thread_factory=lambda: []
            )
        }

    def initialize(self) -> None:
        """Lance l'étape initiale par défaut."""
        if self.current_step in self.steps:
            self.steps[self.current_step].start(self.robot)

    def transition_to(self, new_step: str) -> None:
        """Arrête proprement l'ancienne étape et bascule sur la nouvelle."""
        if new_step == self.current_step or new_step not in self.steps:
            return
        self.steps[self.current_step].stop()
        self.current_step = new_step
        self.steps[self.current_step].start(self.robot)

    def shutdown_all(self) -> None:
        """Force l'arrêt de tous les gestionnaires d'étapes."""
        for step_config in self.steps.values():
            step_config.stop()


# ── FONCTIONS POUR FLASK ET CAPTURE LIVE INDÉPENDANTE ─────────────────────────

def thread_global_camera_capture(camera_instance: Picamera2, log_instance):
    """Met à jour en tâche de fond l'image brute de la caméra et détecte le rouge en bas."""
    global latest_frame, system_running, target_step
    while system_running:
        try:
            frame = camera_instance.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # --- DÉTECTION DU ROUGE DANS LE BAS DE L'IMAGE ---
            height, width = frame_bgr.shape[:2]
            # Définition de la ROI (20% inférieur de l'image)
            roi_top = int(height * 0.80)
            roi = frame_bgr[roi_top:height, 0:width]

            # Conversion HSV et masquage du rouge
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)

            red_pixels = cv2.countNonZero(red_mask)
            if red_pixels > 400 and target_step != "Camera Line":
                log_instance.info(f"    Rouge détecté en bas --> bascule automatique vers Camera Line.")
                target_step = "Camera Line"
            cv2.rectangle(frame_bgr, (0, roi_top), (width, height), (0, 0, 255) if red_pixels > 400 else (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"Zone Rouge: {red_pixels}px", (10, roi_top - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255) if red_pixels > 400 else (0, 255, 0), 1)

            with frame_lock:
                latest_frame = frame_bgr.copy()
        except Exception:
            pass
        time.sleep(0.04)


def generate_global_frames():
    """Générateur de flux MJPEG pour Flask."""
    global latest_frame, system_running
    while system_running:
        with frame_lock:
            if latest_frame is None:
                img_bytes = None
            else:
                _, enc = cv2.imencode('.jpg', latest_frame)
                img_bytes = enc.tobytes()

        if img_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + img_bytes + b'\r\n')
        time.sleep(0.05)


@app_global.route('/')
def index():
    """Interface HTML épurée pour la supervision globale."""
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Supervision Globale - Team C</title>
        <style>
            body { font-family: sans-serif; background: #121214; color: #e1e1e6; text-align: center; padding: 20px; }
            h1 { color: #04d361; }
            .container { max-width: 700px; margin: 0 auto; background: #202024; padding: 20px; border-radius: 8px; box-shadow: 0 4px 10px rgba(0,0,0,0.3); }
            img { width: 100%; border-radius: 4px; border: 2px solid #29292e; max-width: 640px; }
            .info { margin-top: 15px; font-size: 1.1em; color: #a8a8b3; }
        </style>
    </head>
    <body>
        <div class="container">
            <h1>Cockpit de Vidéo-Surveillance Live</h1>
            <img src="/video_feed">
            <div class="info">Serveur Principal de Flux Brut - Robot Team C</div>
        </div>
    </body>
    </html>
    """)


@app_global.route('/video_feed')
def video_feed():
    return Response(generate_global_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ── CONFIGURATION DES ENTRÉES CLAVIER ASYNCHRONES VIA INPUT() ─────────────────

def thread_keyboard_listener(robot_instance: Robot, manager: RobotStepManager, log_instance):
    """Écoute la console en arrière-plan sans bloquer la boucle principale du programme."""
    global system_running, target_step
    print("\n" + "=" * 60)
    print(" CONTRÔLES CLAVIER DISPONIBLES :")
    print(" -> Taper [start]    : Lancer / Continuer le robot")
    print(" -> Taper [stop]     : Arrêter immédiatement le robot (Moteurs OFF)")
    print(" -> [Entrée (vide)]  : Intervertir l'état actuel (Arrêt/Continuer)")
    print(" -> Appuyez sur [1]  : Passer au mode 'Line following'")
    print(" -> Appuyez sur [2]  : Passer au mode 'Labyrinthe'")
    print(" -> Appuyez sur [3]  : Passer au mode 'Camera Line'")
    print(" -> Appuyez sur [4]  : Passer au mode 'Obstacles'")
    print("=" * 60 + "\n")

    while system_running:
        try:
            # Utilisation directe et propre de input() natif
            user_input = input().strip().lower()

            # Ordre d'arrêt explicite
            if user_input == "stop":
                with robot_instance.state.lock:
                    robot_instance.state.emergency_stop = True
                robot_instance.motor.stop()
                log_instance.warning("🛑 CLAVIER : ARRÊT DU ROBOT EFFECTUÉ")

            # Ordre de démarrage explicite
            elif user_input == "start":
                with robot_instance.state.lock:
                    robot_instance.state.emergency_stop = False
                log_instance.info(" CLAVIER : DÉMARRAGE ET ENVOI DU ROBOT")

            # Commutation par touche entrée vide (Toggle)
            elif user_input == "":
                with robot_instance.state.lock:
                    current_estop = robot_instance.state.emergency_stop
                    robot_instance.state.emergency_stop = not current_estop

                    if robot_instance.state.emergency_stop:
                        log_instance.warning("  CLAVIER -> ARRÊT (Moteurs stoppés)")
                        robot_instance.motor.stop()
                    else:
                        log_instance.info(" CLAVIER -> CONTINUER (Moteurs réarmés)")

            # Basculement vers les différentes étapes du robot
            elif user_input in manager.step_mapping:
                next_step = manager.step_mapping[user_input]
                target_step = next_step
                log_instance.info(f"🔄 Demande de transition clavier reçue -> Mode : '{next_step}'")

        except (IOError, EOFError):
            break
        except Exception as e:
            log_instance.error(f"Erreur d'écoute clavier : {e}")
        time.sleep(0.1)


# ── POINT D'ENTRÉE PRINCIPAL D'EXÉCUTION ──────────────────────────────────────

if __name__ == "__main__":
    log = get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║  Robot Line Follower — Team C — SE 2026      ║")
    log.info("╚══════════════════════════════════════════════╝")

    # Initialisation globale du système matériel
    args = parse_args()
    robot = Robot(args)
    robot.init()

    # Instance unique de la caméra partagée
    camera = Picamera2()
    camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
    camera.start()

    # Instanciation de la machine à états
    step_manager = RobotStepManager(robot, camera, args)
    step_manager.initialize()

    # Threads transverses globaux
    global_threads = [
        threading.Thread(target=thread_global_camera_capture, args=(camera, log), name="GLOBAL_CAM", daemon=True),
        threading.Thread(target=thread_keyboard_listener, args=(robot, step_manager, log), name="KEYBOARD",
                         daemon=True),
        threading.Thread(
            target=lambda: app_global.run(host="0.0.0.0", port=5001, debug=False, threaded=True, use_reloader=False),
            name="WEB_GLOBAL", daemon=True
        )
    ]

    for gt in global_threads:
        gt.start()

    log.info("📡 Serveur global de streaming disponible sur http://localhost:5001")

    try:
        while True:
            if step_manager.current_step != target_step:
                log.info(f"Transition vers l'étape : {target_step}")
                step_manager.transition_to(target_step)

            time.sleep(0.1)

    except KeyboardInterrupt:
        log.warning("Interruption utilisateur détectée (Ctrl+C).")

    finally:
        log.info("Arrêt global du robot et nettoyage des ressources...")
        system_running = False

        with robot.state.lock:
            robot.state.running = False
        step_manager.shutdown_all()

        try:
            camera.stop()
            camera.close()
        except Exception:
            pass

        robot.shutdown()
        log.info("Système correctement arrêté.")