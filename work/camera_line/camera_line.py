import time
import threading
import subprocess
from enum import Enum, auto
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# Imports spécifiques au matériel de votre Robot
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

from line_following.t11_argument_parser import parse_args
from line_following.t11_robot import Robot

class LinePosition(Enum):
    STRAIGHT = auto()
    TURN_LEFT_SOFT = auto()
    TURN_RIGHT_SOFT = auto()
    TURN_LEFT_HARD = auto()
    TURN_RIGHT_HARD = auto()
    INTERSECTION = auto()
    LINE_LOST = auto()

class Direction:
    FORWARD = "forward"
    BACKWARD = "backward"

# ── PARAMÈTRES DE CONFIGURATION REGLABLES ─────────────────────────────────────
SPEED_NORMAL_PCT  = 35  
SPEED_TURNING_PCT = 30  
SPEED_SLOW_PCT    = 26  

STEER_CENTER_DEG  = 90
STEER_SOFT_DEG    = 15
STEER_HARD_DEG    = 35

MIN_LINE_AREA  = 300  
CTRL_INTERVAL  = 0.05

# --- PARAMÈTRES ANTI-ZIGZAG ---
THRESHOLD_DEADZONE = 10  # En dessous de 10px d'écart, le robot reste parfaitement droit
THRESHOLD_SOFT     = 25  # Seuil pour virage léger
THRESHOLD_HARD     = 55  # Seuil pour virage serré

ALPHA_SMOOTHING    = 0.4 # Plus cette valeur est basse, plus les mouvements sont lissés (amortisseur)

# Variable globale pour stocker l'erreur lissée entre deux images
smoothed_error = 0.0

lock = threading.Lock()
telemetry = {
    "fps":           0.0,
    "line_seen":     "NON",
    "error_px":      0,
    "stable_dir":    "AUCUNE"
}

system_running = True
app = Flask(__name__)


def process_frame(frame: np.ndarray, robot_instance: Robot) -> np.ndarray:
    """Isole la ligne rouge au plus bas, lisse l'erreur et applique la deadzone."""
    global smoothed_error
    height, width = frame.shape[:2]
    center_x = width // 2

    # Définition de la ROI sur le bas de l'image (30% derniers pixels)
    roi_top = int(height * 0.70)
    roi = frame[roi_top:height, 0:width]

    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv_roi, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv_roi, lower_red2, upper_red2)
    mask_roi = cv2.bitwise_or(mask1, mask2)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    mask_roi = cv2.morphologyEx(mask_roi, cv2.MORPH_OPEN, kernel)

    M = cv2.moments(mask_roi)
    
    output = frame.copy()
    cv2.line(output, (0, roi_top), (width, roi_top), (0, 140, 255), 1)
    cv2.line(output, (center_x, 0), (center_x, height), (255, 0, 0), 2)

    current_action = LinePosition.LINE_LOST
    line_seen = "NON"
    error_instax = 0
    stable_dir = "LIGNE PERDUE"
    color_text = (0, 0, 255)

    if M["m00"] > MIN_LINE_AREA:
        line_seen = "OUI"
        cx = int(M["m10"] / M["m00"])
        cy_global = roi_top + int(M["m01"] / M["m00"])
        
        # Erreur brute de l'image actuelle
        error_instax = cx - center_x

        # Filtrage/Amortissement
        smoothed_error = (ALPHA_SMOOTHING * error_instax) + ((1.0 - ALPHA_SMOOTHING) * smoothed_error)
        error = int(smoothed_error)

        cv2.circle(output, (cx, cy_global), 8, (0, 255, 0), -1)

        # Logique de décision avec ZONE MORTE (Anti-oscillation)
        if abs(error) <= THRESHOLD_DEADZONE:
            current_action = LinePosition.STRAIGHT
            stable_dir = f"TOUT DROIT (Zone Morte {error}px)"
            color_text = (0, 255, 0)
        elif error < -THRESHOLD_HARD:
            current_action = LinePosition.TURN_LEFT_HARD
            stable_dir = "VIRAGE GAUCHE FORTE"
            color_text = (0, 165, 255)
        elif error < -THRESHOLD_SOFT:
            current_action = LinePosition.TURN_LEFT_SOFT
            stable_dir = "VIRAGE GAUCHE LEGER"
            color_text = (255, 255, 0)
        elif error > THRESHOLD_HARD:
            current_action = LinePosition.TURN_RIGHT_HARD
            stable_dir = "VIRAGE DROITE FORTE"
            color_text = (0, 165, 255)
        elif error > THRESHOLD_SOFT:
            current_action = LinePosition.TURN_RIGHT_SOFT
            stable_dir = "VIRAGE DROITE LEGER"
            color_text = (255, 255, 0)
        else:
            current_action = LinePosition.STRAIGHT
            stable_dir = "TOUT DROIT"
            color_text = (0, 255, 0)

        cv2.putText(output, f"Ecart Lisse: {error}px (Brut: {error_instax}px)", (10, height - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.putText(output, f"ORDRE : {stable_dir}", (10, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_text, 2)

    with robot_instance.state.lock:
        robot_instance.state.line_action = current_action

    with lock:
        telemetry["line_seen"]  = line_seen
        telemetry["error_px"]   = int(smoothed_error)
        telemetry["stable_dir"] = stable_dir

    return output


def thread_controller_camera_line(robot: Robot, interval: float) -> None:
    """Boucle de décision : Applique les vitesses modérées et les angles de roues."""
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency = robot.state.emergency_stop
            action    = robot.state.line_action

        if emergency:
            robot.motor.stop()
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)
            time.sleep(interval)
            continue

        if action == LinePosition.STRAIGHT:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)

        elif action == LinePosition.TURN_LEFT_SOFT:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG + STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

        elif action == LinePosition.TURN_RIGHT_SOFT:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG - STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

        elif action == LinePosition.TURN_LEFT_HARD:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG + STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT, fast_accel=True)

        elif action == LinePosition.TURN_RIGHT_HARD:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG - STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_SLOW_PCT, fast_accel=True)

        elif action == LinePosition.INTERSECTION:
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)

        else: 
            robot.motor.stop()
            robot.head.set_angle_motor(0, STEER_CENTER_DEG)

        time.sleep(interval)

    robot.motor.stop()
    robot.head.set_angle_motor(0, STEER_CENTER_DEG)



def generate_frames(robot_instance: Robot):
    global system_running
    
    cmd = [
        "rpicam-vid", "-t", "0", "--inline",
        "--width", "640", "--height", "480",
        "--framerate", "30", "--codec", "mjpeg", "-o", "-"
    ]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buf = b""

    frame_count = 0
    t0 = time.time()

    try:
        while system_running:
            chunk = process.stdout.read(4096)
            if not chunk: 
                break
            buf += chunk

            a = buf.find(b"\xff\xd8")
            b_e = buf.find(b"\xff\xd9")
            if a != -1 and b_e > a:
                jpg = buf[a: b_e + 2]
                buf = buf[b_e + 2:]

                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is None: 
                    continue

                frame_count += 1
                elapsed = time.time() - t0
                if elapsed >= 1.0:
                    with lock:
                        telemetry["fps"] = round(frame_count / elapsed, 1)
                    frame_count, t0 = 0, time.time()

                processed = process_frame(frame, robot_instance)
                _, enc = cv2.imencode(".jpg", processed)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + enc.tobytes() + b"\r\n")
    except Exception:
        pass
    finally:
        process.terminate()


HTML_INTERFACE = """<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team C - Monitor Stabilisé</title>
<style>
:root{--bg:#070a12;--panel:#0e131f;--border:#1f293d;--text:#b2c0d6;--cyan:#00e5ff;--red:#ff3d57;--green:#00e676;--mono:monospace}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:var(--bg);color:var(--text);padding:20px}
header{text-align:center;margin-bottom:20px}
header h1{font-size:1.3rem;color:var(--cyan);letter-spacing:1px}
.layout{display:flex;flex-wrap:wrap;gap:20px;justify-width:center;max-width:1000px;margin:auto}
.vbox{position:relative;border:2px solid var(--border);border-radius:8px;overflow:hidden;width:640px;background:#000}
.vbox img{display:block;width:100%}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:20px;width:320px;display:flex;flex-direction:column;gap:12px}
.ptitle{font-size:.65rem;text-transform:uppercase;color:#4f637d;border-bottom:1px solid var(--border);padding-bottom:5px}
.row{display:flex;justify-content:space-between;font-size:.85rem}
.val{font-family:var(--mono);font-weight:700;color:#fff}
.dir-main{text-align:center;font-size:1.4rem;font-weight:900;font-family:var(--mono);margin:10px 0}
</style>
<script>
async function tick(){
  const d=await fetch('/data').then(r=>get_json(r));
  async function get_json(r){try{return await r.json()}catch(e){return{}}}
  if(!d || !d.fps) return;
  document.getElementById('fps').textContent=d.fps+' Hz';
  document.getElementById('seen').textContent=d.line_seen;
  document.getElementById('err').textContent=d.error_px+' px';
  
  const el=document.getElementById('dir-val');
  el.textContent=d.stable_dir;
  if(d.stable_dir.includes('GAUCHE')) el.style.color='#00e5ff';
  else if(d.stable_dir.includes('DROITE')) el.style.color='#ff3d57';
  else if(d.stable_dir.includes('DROIT')) el.style.color='#00e676';
  else el.style.color='#697c98';
  
  document.getElementById('vbox').style.borderColor=el.style.color;
}
setInterval(tick,100);
</script>
</head>
<body>
<header>
  <h1>🤖 Robot Monitor — Anti-Oscillation & Max 30% — SE 2026</h1>
</header>
<div class="layout">
  <div class="vbox" id="vbox"><img src="/video_feed"></div>
  <div class="panel">
    <div class="ptitle">Données Télémétrie</div>
    <div class="row"><span>Fréquence Image</span><span class="val" id="fps">— Hz</span></div>
    <div class="row"><span>Ligne Détectée ?</span><span class="val" id="seen">NON</span></div>
    <div class="row"><span>Écart de Suivi (Lissé)</span><span class="val" id="err">0 px</span></div>
    <div class="ptitle">Ordre Actif Moteurs</div>
    <div class="dir-main" id="dir-val">RECHERCHE...</div>
  </div>
</div>
</body>
</html>"""

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

# POINT D'ENTRÉE PRINCIPAL

if __name__ == "__main__":
    args = parse_args()

    subprocess.run(["sudo", "pkill", "-f", "rpicam"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)

    robot = Robot(args)
    robot.init()
    
    robot.head.set_angle_motor(2, 60)
    
    robot.state.line_action = LinePosition.LINE_LOST
    global_robot_ref = robot 

    threads = [
        threading.Thread(target=thread_controller_camera_line, args=(robot, CTRL_INTERVAL), name="CTRL", daemon=True),
        threading.Thread(target=lambda: app.run(host="0.0.0.0", port=5000, debug=False, threaded=True, use_reloader=False), name="WEB", daemon=True)
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