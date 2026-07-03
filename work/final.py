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

# Références globales requises pour les routes Flask
robot = None
step_manager = None
log = None


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

        self.step_mapping = {
            "1": "Line following",
            "2": "Obstacles",
            "3": "Labyrinthe",
            "4": "Flèches"
        }

        self.steps: Dict[str, StepConfig] = {
            "Line following": StepConfig(
                camera_angle=90,
                thread_factory=lambda: [
                    threading.Thread(target=t11_thread_ultrasonic, args=(robot_instance, args_instance.sensor_interval),
                                     name="US_IR", daemon=True),
                    threading.Thread(target=t11_thread_line, args=(robot_instance, args_instance.sensor_interval),
                                     name="LINE_IR", daemon=True),
                    threading.Thread(target=t11_thread_LED, args=(robot_instance, args_instance.sensor_interval),
                                     name="LED_IR", daemon=True),
                    threading.Thread(target=t11_thread_controller, args=(robot_instance, args_instance.ctrl_interval),
                                     name="CTRL_IR", daemon=True),
                    threading.Thread(target=t11_thread_buzzer, args=(robot_instance,), name="BUZZER", daemon=True),
                ]
            ),
            "Obstacles": StepConfig(
                camera_angle=90,
                thread_factory=lambda: []
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
                    threading.Thread(target=thread_camera_loop, args=(robot_instance,), name="CAM_AUTO", daemon=True),
                    threading.Thread(
                        target=lambda: app_camera_line.run(host="0.0.0.0", port=5000, debug=False, threaded=True,
                                                           use_reloader=False),
                        name="WEB_CAM_LINE", daemon=True
                    )
                ]
            )
        }

    def initialize(self) -> None:
        if self.current_step in self.steps:
            self.steps[self.current_step].start(self.robot)

    def transition_to(self, new_step: str) -> None:
        if new_step == self.current_step or new_step not in self.steps:
            return
        self.steps[self.current_step].stop()
        self.current_step = new_step
        self.steps[self.current_step].start(self.robot)

    def shutdown_all(self) -> None:
        for step_config in self.steps.values():
            step_config.stop()


# ── FONCTIONS POUR FLASK ET CAPTURE LIVE PURE ─────────────────────────────────

def thread_global_camera_capture(camera_instance: Picamera2, log_instance):
    global latest_frame, system_running
    while system_running:
        try:
            frame = camera_instance.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with frame_lock:
                latest_frame = frame_bgr.copy()
        except Exception:
            pass
        time.sleep(0.04)


def generate_global_frames():
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
        <title>Supervision Globale - Team C</title>
        <meta name="viewport" content="width=device-width, initial-scale=1">
        <style>
            body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121214; color: #e1e1e6; text-align: center; padding: 20px; margin: 0; }
            h1 { color: #04d361; margin-bottom: 20px; }
            .container { max-width: 750px; margin: 0 auto; background: #202024; padding: 25px; border-radius: 8px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
            .video-box { position: relative; display: inline-block; width: 100%; max-width: 640px; }
            img { width: 100%; border-radius: 6px; border: 2px solid #29292e; background: #000; }

            .section-title { font-size: 1.1em; color: #04d361; text-transform: uppercase; letter-spacing: 1px; margin: 20px 0 10px 0; font-weight: bold;}
            .btn-group { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; margin-bottom: 15px; }

            button { background: #29292e; color: #e1e1e6; border: 2px solid #3e3e44; padding: 12px 24px; font-size: 14px; font-weight: bold; border-radius: 6px; cursor: pointer; transition: all 0.2s ease; min-width: 140px; }
            button:hover { background: #3e3e44; border-color: #04d361; }
            button:active { transform: scale(0.98); }

            button.btn-start { background: #1b4d22; border-color: #2e7d32; color: #a5d6a7; }
            button.btn-start:hover { background: #2e7d32; }
            button.btn-stop { background: #661a1a; border-color: #c62828; color: #ef9a9a; }
            button.btn-stop:hover { background: #c62828; }

            .status-panel { background: #1a1a1e; padding: 12px; border-radius: 6px; margin-top: 15px; border: 1px solid #29292e; display: flex; justify-content: space-around; font-size: 0.95em; }
            .status-val { color: #04d361; font-weight: bold; }
        </style>
        <script>
            function sendCommand(endpoint, param='') {
                let url = endpoint + (param ? '?mode=' + param : '');
                fetch(url, { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    updateUI(data);
                })
                .catch(err => console.error('Erreur:', err));
            }

            function updateUI(data) {
                document.getElementById('current-mode-status').innerText = data.current_step;
                document.getElementById('motor-status').innerText = data.robot_running ? "ACTIF" : "ARRÊTÉ (running=false)";
                document.getElementById('motor-status').style.color = data.robot_running ? "#04d361" : "#ef9a9a";
            }

            setInterval(() => {
                fetch('/status')
                .then(res => res.json())
                .then(data => updateUI(data))
                .catch(err => console.error(err));
            }, 1000);
        </script>
    </head>
    <body>
        <div class="container">
            <h1>Cockpit de Contrôle — Team C</h1>

            <div class="video-box">
                <img src="/video_feed" alt="Flux vidéo live">
            </div>

            <div class="status-panel">
                <div>Statut Global Robot: <span id="motor-status" class="status-val">--</span></div>
                <div>Mode Actif: <span id="current-mode-status" class="status-val">--</span></div>
            </div>

            <div class="section-title">Commandes Générales</div>
            <div class="btn-group">
                <button class="btn-start" onclick="sendCommand('/control/start')">START (running=true)</button>
                <button class="btn-stop" onclick="sendCommand('/control/stop')">STOP (running=false)</button>
            </div>

            <div class="section-title">Sélection du Mode (Circuit)</div>
            <div class="btn-group">
                <button onclick="sendCommand('/control/mode', '1')">Line Following</button>
                <button onclick="sendCommand('/control/mode', '2')">Obstacles</button>
                <button onclick="sendCommand('/control/mode', '3')">Labyrinthe</button>
                <button onclick="sendCommand('/control/mode', '4')">Flèches</button>
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
    log.info("🌐 WEB : robot.state.running = True")
    return jsonify({"status": "success", "robot_running": True, "current_step": step_manager.current_step})


@app_global.route('/control/stop', methods=['POST'])
def web_stop():
    global robot, log
    with robot.state.lock:
        robot.state.running = False
    robot.motor.stop()
    log.warning("🛑 WEB : robot.state.running = False (Moteurs coupés)")
    return jsonify({"status": "success", "robot_running": False, "current_step": step_manager.current_step})


@app_global.route('/control/mode', methods=['POST'])
def web_change_mode():
    global step_manager, target_step, log
    mode_id = request.args.get('mode')
    if mode_id in step_manager.step_mapping:
        next_step = step_manager.step_mapping[mode_id]
        target_step = next_step
        log.info(f"🔄 WEB : Transition manuelle -> Mode : '{next_step}'")
        return jsonify({"status": "success", "robot_running": robot.state.running, "current_step": next_step})
    return jsonify({"status": "error", "message": "Mode invalide"}), 400


# ── POINT D'ENTRÉE PRINCIPAL D'EXÉCUTION ──────────────────────────────────────

if __name__ == "__main__":
    log = get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║  Robot Line Follower — Team C — SE 2026      ║")
    log.info("╚══════════════════════════════════════════════╝")

    args = parse_args()
    robot = Robot(args)
    robot.init()

    camera = Picamera2()
    camera.configure(camera.create_video_configuration(main={"size": (640, 480)}))
    camera.start()

    step_manager = RobotStepManager(robot, camera, args)
    step_manager.initialize()

    global_threads = [
        threading.Thread(target=thread_global_camera_capture, args=(camera, log), name="GLOBAL_CAM", daemon=True),
        threading.Thread(
            target=lambda: app_global.run(host="0.0.0.0", port=5001, debug=False, threaded=True, use_reloader=False),
            name="WEB_GLOBAL", daemon=True
        )
    ]

    for gt in global_threads:
        gt.start()

    log.info("📡 Serveur de contrôle actif sur http://localhost:5001")

    try:
        while True:
            # Gestion des changements d'états demandés par la page web
            if step_manager.current_step != target_step:
                log.info(f"Transition vers l'étape : {target_step}")
                step_manager.transition_to(target_step)

            time.sleep(0.1)

    except KeyboardInterrupt:
        log.warning("Interruption détectée.")

    finally:
        log.info("Arrêt global du robot...")
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