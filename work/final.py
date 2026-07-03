import threading
import time
import sys
import cv2
import numpy as np
from typing import Dict, List, Callable
from flask import Flask, Response, render_template_string, request, jsonify

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

# Threads — Suivi de Ligne Via Caméra Autonome (Flèches)
from camera_line3 import (
    thread_controller_camera_line as thread_camera_line,
    CTRL_INTERVAL,
    thread_ultrasonic as thread_camera_line_US,
    US_INTERVAL,
    thread_LED as thread_camera_line_LED,
    LED_INTERVAL,
    thread_camera_loop as cam3_thread_camera_loop,
    app as app_camera_line
)
from transitions import *

import camera_line3

# Threads - Transition Line Following
from TransitionLineFollowing import (
    thread_controller_camera_line as trans_thread_controller,
    thread_ultrasonic as trans_thread_ultrasonic,
    thread_LED as trans_thread_LED,
    thread_camera_loop as trans_thread_camera_loop
)

from avoid_objects_threads import thread_ultrasonic_scanning, thread_object_controller, thread_line_detect_avoid, thread_avoid_line_controller


frame_lock = threading.Lock()
latest_frame = None
system_running = True
# L'état initial cible
target_step = "Transition Line following"

# Configuration du serveur Flask global pour la supervision (Port 5001)
app_global = Flask(__name__)

# Références globales requises pour les routes Flask
robot = None
step_manager = None
log = None


def run_calibration_and_route(calib_func: Callable[[Robot], str], robot_instance: Robot) -> None:
    """Exécute une fonction de calibration et intercepte son retour pour mettre à jour target_step."""
    global target_step
    next_step = calib_func(robot_instance)
    if next_step:
        target_step = next_step


class StepConfig:
    """Structure de données pour configurer chaque étape du robot."""

    def __init__(self, camera_angle: int, thread_factory: Callable[[], List[threading.Thread]]):
        self.camera_angle = camera_angle
        self.thread_factory = thread_factory
        self.active_threads: List[threading.Thread] = []

    def start(self, robot_instance: Robot) -> None:
        """oriente la caméra, génère les threads et les lance."""
        robot_instance.head.set_angle_motor(2, self.camera_angle)
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

    def __init__(self, robot_instance: Robot, camera_instance: Picamera2, args_instance):
        self.robot = robot_instance
        self.camera = camera_instance
        self.args = args_instance
        self.current_step: str = "Line following"

        # Mapping mis à jour pour correspondre à votre nouveau cycle manuel
        self.step_mapping = {
            "1": "Line following",
            "2": "Obstacles",
            "3": "Labyrinthe",
            "4": "Camera Line",
            "5": "Calibration Obstacles",
            "6": "Calibration Ligne Rouge",
            "7": "Calibration Labyrinthe"
        }

        # Définition des stratégies de chaque étape
        self.steps: Dict[str, StepConfig] = {
            "Line following": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=t11_thread_ultrasonic, args=(robot_instance, args_instance.sensor_interval),
                                     name="US_IR", daemon=True),
                    threading.Thread(target=t11_thread_line, args=(robot_instance, args_instance.sensor_interval),
                                     name="LINE_IR", daemon=True),
                    threading.Thread(target=t11_thread_controller, args=(robot_instance, args_instance.ctrl_interval),
                                     name="CTRL_IR", daemon=True),
                    threading.Thread(target=t11_thread_buzzer, args=(robot_instance,), name="BUZZER", daemon=True),
                ]
            ),
            "Calibration Obstacles": StepConfig(
                camera_angle=90,
                thread_factory=lambda: [
                    threading.Thread(target=run_calibration_and_route,
                                     args=(calibration_sequence_IR_to_obstacles, robot_instance),
                                     name="CALIB_OBST", daemon=True)
                ]
            ),
            "Obstacles": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=thread_ultrasonic_scanning,
                                     args=(robot_instance, args_instance.sensor_interval),
                                     name="US", daemon=True),
                    threading.Thread(target=thread_object_controller,
                                     args=(robot_instance, args_instance.sensor_interval),
                                     name="CTRL_OBJ", daemon=True),
                    threading.Thread(target=thread_line_detect_avoid,
                                     args=(robot_instance, args_instance.sensor_interval),
                                     name="LINE_DETECT", daemon=True),
                    threading.Thread(target=thread_avoid_line_controller,
                                     args=(robot_instance, args_instance.sensor_interval),
                                     name="CTRL_LINE", daemon=True)
                ]
            ),
            "Calibration Ligne Rouge": StepConfig(
                camera_angle=90,
                thread_factory=lambda: [
                    threading.Thread(target=run_calibration_and_route,
                                     args=(calibration_sequence_obstacles_to_camera_line, robot_instance),
                                     name="CALIB_ROUGE", daemon=True)
                ]
            ),
            "Camera Line": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=camera_line3.thread_controller_camera_line,
                                     args=(robot_instance, camera_line3.CTRL_INTERVAL), name="CTRL", daemon=True),
                    threading.Thread(target=camera_line3.thread_ultrasonic,
                                     args=(robot_instance, camera_line3.US_INTERVAL), name="US", daemon=True),
                    threading.Thread(target=camera_line3.thread_camera_loop, args=(robot_instance, self.camera),
                                     name="CAM_AUTO", daemon=True),
                    threading.Thread(
                        target=lambda: camera_line3.app.run(host="0.0.0.0", port=5002, debug=False, threaded=True,
                                                            use_reloader=False),
                        name="WEB_CAM_LINE", daemon=True
                    )
                ]
            ),
            "Calibration Labyrinthe": StepConfig(
                camera_angle=90,
                thread_factory=lambda: [
                    threading.Thread(target=run_calibration_and_route,
                                     args=(calibration_sequence_camera_to_labyrinth, robot_instance),
                                     name="CALIB_LABY", daemon=True)
                ]
            ),
            "Labyrinthe": StepConfig(
                camera_angle=110,
                thread_factory=lambda: [
                    threading.Thread(target=labyrinthe_thread_ultrasonic,
                                     args=(robot_instance, args_instance.sensor_interval), name="US_Labyrinthe",
                                     daemon=True),
                    threading.Thread(target=labyrinthe_thread_drive,
                                     args=(robot_instance, args_instance.sensor_interval, self.camera),
                                     name="Camera_Labyrinthe", daemon=True)
                ]
            ),
            "Flèches": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=thread_camera_line, args=(robot_instance, CTRL_INTERVAL), name="CTRL",
                                     daemon=True),
                    threading.Thread(target=thread_camera_line_US, args=(robot_instance, US_INTERVAL), name="US",
                                     daemon=True),
                    threading.Thread(target=thread_camera_line_LED, args=(robot_instance, LED_INTERVAL), name="LED",
                                     daemon=True),
                    threading.Thread(target=cam3_thread_camera_loop, args=(robot_instance, self.camera),
                                     name="CAM_AUTO", daemon=True),
                    threading.Thread(target=lambda: app_camera_line.run(host="0.0.0.0", port=5000, debug=False,
                                                                        threaded=True, use_reloader=False),
                                     name="WEB_CAM_LINE", daemon=True)
                ]
            ),
            "Transition Line following": StepConfig(
                camera_angle=60,
                thread_factory=lambda: [
                    threading.Thread(target=trans_thread_controller, args=(robot_instance, CTRL_INTERVAL), name="CTRL",
                                     daemon=True),
                    threading.Thread(target=trans_thread_ultrasonic, args=(robot_instance, US_INTERVAL), name="US",
                                     daemon=True),
                    threading.Thread(target=trans_thread_LED, args=(robot_instance, LED_INTERVAL), name="LED",
                                     daemon=True),
                    threading.Thread(target=trans_thread_camera_loop, args=(robot_instance, self.camera),
                                     name="CAM_AUTO", daemon=True),
                ]
            ),
        }

    def initialize(self) -> None:
        """Lance l'étape initiale par défaut."""
        if self.current_step in self.steps:
            self.steps[self.current_step].start(self.robot)

    def transition_to(self, new_step: str) -> None:
        """Arrête proprement l'ancienne étape et bascule sur la nouvelle."""
        if new_step == self.current_step or new_step not in self.steps:
            return

        with self.robot.state.lock:
            self.robot.state.running = False
            # Synchro de l'action interne pour éviter le rebond d'état
            self.robot.state.action = new_step

        self.steps[self.current_step].stop()

        if new_step == "Camera Line":
            camera_line3.global_camera_ref = self.camera
            camera_line3.global_robot_ref = self.robot

        is_calibration = "Calibration" in new_step
        with self.robot.state.lock:
            self.robot.state.running = not is_calibration
            self.robot.state.emergency_stop = False

        self.current_step = new_step
        self.steps[self.current_step].start(self.robot)

    def shutdown_all(self) -> None:
        """Force l'arrêt de tous les gestionnaires d'étapes."""
        for step_config in self.steps.values():
            step_config.stop()


# ── FONCTION UNIFIÉE : CAMÉRA GLOBALE ET SUPERVISION D'ÉTAT ──────────────────

def thread_global_camera_and_state(camera_instance: Picamera2, log_instance, robot_instance: Robot):
    """
    Tâche de fond qui met à jour l'image brute, analyse les balises bleues,
    et écoute les requêtes internes de changement d'état (ex: robot.state.action).
    """
    global latest_frame, system_running, target_step, step_manager

    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])

    while system_running:
        # --- PARTIE A : CAPTURE CAMÉRA ET ANALYSE ---
        try:
            frame = camera_instance.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            with frame_lock:
                latest_frame = frame_bgr.copy()

            roi_bottom = frame_bgr[450:480, :]
            hsv = cv2.cvtColor(roi_bottom, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            blue_pixels = cv2.countNonZero(mask)

            if blue_pixels > 400:
                current = step_manager.current_step if step_manager else ""

                if current == "Line following" and target_step == "Line following":
                    log_instance.warning(
                        f"[CV] BALISE_DETECTEE: Masse bleue en bas ({blue_pixels}px) -> Interception Suivi Ligne IR")
                    target_step = "Calibration Obstacles"

                elif current == "Obstacles" and target_step == "Obstacles":
                    log_instance.warning(
                        f"[CV] BALISE_DETECTEE: Masse bleue en bas ({blue_pixels}px) -> Interception Obstacles")
                    target_step = "Calibration Ligne Rouge"

        except Exception:
            pass

        # --- PARTIE B : LECTURE DE L'ÉTAT DU ROBOT (Demandes internes) ---
        with robot_instance.state.lock:
            action = getattr(robot_instance.state, 'action', "")

        current = step_manager.current_step if step_manager else ""
        # Si un thread a modifié l'action et qu'on n'est pas déjà en train de la faire
        if action and action != current and action != target_step:
            log_instance.info(f"[STATE] Changement d'état interne demandé : {action}")
            target_step = action

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
    return render_template_string("""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Cockpit Team C - SE 2026</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: 'Segoe UI', system-ui, sans-serif; background: #0f0f11; color: #e1e1e6; margin: 0; padding: 20px; display: flex; justify-content: center; }
            .container { width: 100%; max-width: 540px; background: #17171a; padding: 20px; border-radius: 12px; box-shadow: 0 8px 24px rgba(0,0,0,0.6); border: 1px solid #242429; }
            h1 { color: #04d361; font-size: 1.5em; margin-top: 0; margin-bottom: 15px; letter-spacing: 0.5px; text-shadow: 0 2px 4px rgba(0,0,0,0.5); }
            .video-box { width: 100%; max-width: 480px; margin: 0 auto 15px auto; overflow: hidden; border-radius: 8px; border: 2px solid #242429; box-shadow: inset 0 2px 4px rgba(0,0,0,0.5); font-size: 0; }
            img { width: 100%; height: auto; aspect-ratio: 4 / 3; background: #000; }
            .status-panel { background: #111112; padding: 12px; border-radius: 8px; margin-bottom: 20px; border: 1px solid #202024; display: flex; justify-content: space-around; font-size: 0.9em; }
            .status-item { display: flex; flex-direction: column; gap: 4px; }
            .status-label { font-size: 0.75em; color: #7c7c8a; text-transform: uppercase; letter-spacing: 0.5px; }
            .status-val { font-weight: bold; font-size: 1.05em; color: #fff; }
            .section-title { font-size: 0.8em; color: #7c7c8a; text-transform: uppercase; letter-spacing: 1px; margin: 0 0 8px 0; text-align: left; font-weight: bold; }
            .btn-group { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; margin-bottom: 20px; }
            button { background: #202024; color: #e1e1e6; border: 1px solid #2e2e35; padding: 14px; font-size: 13px; font-weight: 600; border-radius: 8px; cursor: pointer; transition: all 0.15s ease; }
            button:hover { background: #29292e; border-color: #04d361; }
            button:active { transform: scale(0.97); }
            button.btn-start { background: #12361b; border-color: #1e592c; color: #87f5a9; grid-column: span 1; }
            button.btn-start:hover { background: #1b4d26; }
            button.btn-stop { background: #4a1919; border-color: #732727; color: #fca3a3; grid-column: span 1; }
            button.btn-stop:hover { background: #612222; }
            .btn-group.modes button { text-align: center; }
            .btn-group.modes button.active { border-color: #04d361; background: #242429; color: #04d361; box-shadow: 0 0 8px rgba(4,211,97,0.2); }
        </style>
        <script>
            function sendCommand(endpoint, param='') {
                let url = endpoint + (param ? '?mode=' + param : '');
                fetch(url, { method: 'POST' })
                .then(response => response.json())
                .then(data => updateUI(data))
                .catch(err => console.error('Erreur:', err));
            }
            function updateUI(data) {
                document.getElementById('current-mode-status').innerText = data.current_step;
                const motor = document.getElementById('motor-status');
                motor.innerText = data.robot_running ? "EN MARCHE" : "ARRÊTÉ";
                motor.style.color = data.robot_running ? "#04d361" : "#fca3a3";

                const modes = { "Line following": "m1", "Obstacles": "m2", "Labyrinthe": "m3", "Camera Line": "m4" };
                document.querySelectorAll('.btn-group.modes button').forEach(b => b.classList.remove('active'));
                if (modes[data.current_step]) {
                    document.getElementById(modes[data.current_step]).classList.add('active');
                }
            }
            setInterval(() => {
                fetch('/status').then(res => res.json()).then(data => updateUI(data)).catch(err => console.error(err));
            }, 1000);
        </script>
    </head>
    <body>
        <div class="container">
            <h1>Cockpit Robot — Team C</h1>
            <div class="video-box"><img src="/video_feed" alt="Flux live"></div>
            <div class="status-panel">
                <div class="status-item"><span class="status-label">Moteurs (running)</span><span id="motor-status" class="status-val">--</span></div>
                <div class="status-item"><span class="status-label">Mode Actif</span><span id="current-mode-status" class="status-val">--</span></div>
            </div>
            <div class="section-title">Alimentation Principale</div>
            <div class="btn-group">
                <button class="btn-start" onclick="sendCommand('/control/start')">▶ START</button>
                <button class="btn-stop" onclick="sendCommand('/control/stop')">🛑 STOP</button>
            </div>
            <div class="section-title">Changement de Mode Manuel</div>
            <div class="btn-group modes">
                <button id="m1" onclick="sendCommand('/control/mode', '1')">Line Following</button>
                <button id="m2" onclick="sendCommand('/control/mode', '2')">Obstacles</button>
                <button id="m3" onclick="sendCommand('/control/mode', '3')">Labyrinthe</button>
                <button id="m4" onclick="sendCommand('/control/mode', '4')">Camera Line</button>
            </div>
        </div>
    </body>
    </html>
    """)


@app_global.route('/video_feed')
def video_feed():
    return Response(generate_global_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ── ENDPOINTS API POUR LES BOUTONS WEB ────────────────────────────────────────

@app_global.route('/status', methods=['GET'])
def get_status():
    return jsonify({
        "current_step": step_manager.current_step if step_manager else "Unknown",
        "robot_running": robot.state.running if robot else False
    })


@app_global.route('/control/start', methods=['POST'])
def web_start():
    global robot, log
    with robot.state.lock:
        robot.state.running = True
        robot.state.emergency_stop = False
    log.info("[NET] HTTP_POST: /control/start -> robot.state.running = True")
    return jsonify({"status": "success", "robot_running": True, "current_step": step_manager.current_step})


@app_global.route('/control/stop', methods=['POST'])
def web_stop():
    global robot, log
    with robot.state.lock:
        robot.state.running = False
    robot.motor.stop()
    log.warning("[NET] HTTP_POST: /control/stop -> Arrêt forcé des moteurs")
    return jsonify({"status": "success", "robot_running": False, "current_step": step_manager.current_step})


@app_global.route('/control/mode', methods=['POST'])
def web_change_mode():
    global step_manager, target_step, log
    mode_id = request.args.get('mode')
    if mode_id in step_manager.step_mapping:
        next_step = step_manager.step_mapping[mode_id]
        target_step = next_step
        log.info(f"[NET] HTTP_POST: /control/mode?mode={mode_id} -> Transition forcée vers '{next_step}'")
        return jsonify({"status": "success", "robot_running": robot.state.running, "current_step": next_step})
    return jsonify({"status": "error", "message": "Mode invalide"}), 400


def run_flask_server():
    """Lance le serveur web global sur le port 5001."""
    # Désactiver le logger par défaut de Flask pour éviter de polluer vos logs
    import logging
    logging.getLogger('werkzeug').setLevel(logging.ERROR)

    app_global.run(host="0.0.0.0", port=5001, debug=False, use_reloader=False, threaded=True)


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

    with robot.state.lock:
        robot.state.running = False
        robot.state.action = target_step  # Sync avec target_step

    if not hasattr(robot.state, 'calculated_angle'):
        robot.state.calculated_angle = 90

    camera = Picamera2()
    camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
    camera.start()

    # Instanciation de la machine à états
    step_manager = RobotStepManager(robot, camera, args)
    step_manager.initialize()

    # Déploiement du thread unifié d'observation (Webcam + State Watcher)
    global_threads = [
        threading.Thread(target=thread_global_camera_and_state, args=(camera, log, robot), name="GLOBAL_CAM_STATE",
                         daemon=True),
        threading.Thread(target=lambda: app_global.run(host="0.0.0.0", port=5001, debug=False, threaded=True, use_reloader=False),
                         name="WEB_GLOBAL", daemon=True)
    ]

    for gt in global_threads:
        gt.start()

    log.info("[NET] Serveur Flask en ligne sur http://0.0.0.0:5001")

    lost_line_timestamp = None

    # Boucle de Contrôle (Orchestrateur Principal)
    try:
        while True:
            # 1. Vérifie si un changement d'état est requis
            if step_manager.current_step != target_step:
                log.info(f"[SYS] TRANSITION: '{step_manager.current_step}' -> '{target_step}'")
                step_manager.transition_to(target_step)
                lost_line_timestamp = None

            # 2. Logique spécifique pour la caméra (Keepalive Timeout)
            if step_manager.current_step == "Camera Line":
                with camera_line3.lock:
                    line_seen = camera_line3.telemetry.get("line_seen", "NON")

                if line_seen == "NON":
                    if lost_line_timestamp == None:
                        lost_line_timestamp = time.time()
                    elif time.time() - lost_line_timestamp >= 2.0:
                        log.warning(
                            "[SYS] KEEPALIVE_TIMEOUT: Ligne rouge perdue > 2.0s. Bascule vers Calibration Labyrinthe")
                        target_step = "Calibration Labyrinthe"
                else:
                    lost_line_timestamp = None

            time.sleep(0.05)

    except KeyboardInterrupt:
        log.warning("[SYS] Signal SIGINT reçu, initialisation de la procédure d'arrêt...")

    finally:
        log.info("[SYS] ARRET: Interruption des tâches actives...")
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
        log.info("[SYS] HALT. Système arrêté en toute sécurité.")