import time
import threading
import subprocess
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, jsonify

# Imports spécifiques au matériel de votre Robot
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

from line_following.t11_argument_parser import parse_args
from line_following.t11_robot import Robot

class Direction:
    FORWARD = "forward"
    BACKWARD = "backward"

# ── PARAMÈTRES DE CONFIGURATION REGLABLES ─────────────────────────────────────
SPEED_MAX_PCT     = 48 
SPEED_MIN_PCT     = 37  

STEER_CENTER_DEG  = 90
MAX_STEER_DELTA   = 45  # Braquage max autorisé (90 +/- 45)

MIN_LINE_AREA     = 300  
CTRL_INTERVAL     = 0.05
US_INTERVAL       = 0.06  # Intervalle de rafraîchissement ultrason
LED_INTERVAL      = 0.1   # Intervalle de rafraîchissement des LED


THRESHOLD_DEADZONE = 5     
ALPHA_SMOOTHING    = 0.35  
THRESHOLD_URGENCY  = 30    

# Pondérations nominales
WEIGHT_POSITION    = 0.65  
WEIGHT_DIRECTION   = 0.35  

# Variables de lissage global
smoothed_angle_delta = 0.0

lock = threading.Lock()
telemetry = {
    "fps":           0.0,
    "line_seen":     "NON",
    "error_px":      0,
    "stable_dir":    "AUCUNE",
    "distance_mm":   0,
    "speed_pct":     0,
    "emergency":     False
}

system_running = True
app = Flask(__name__)


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
    """Analyse les bandes à longue distance et court-circuite le lissage en cas de crise."""
    global smoothed_angle_delta
    height, width = frame.shape[:2]
    center_x = width // 2
    output = frame.copy()

    # Remontée agressive des ROIs vers le haut de l'image (Horizon)
    roi_low_top, roi_low_bot   = int(height * 0.70), int(height * 0.90)
    roi_high_top, roi_high_bot = int(height * 0.20), int(height * 0.40)

    mask_low  = get_red_mask(frame[roi_low_top:roi_low_bot, 0:width])
    mask_high = get_red_mask(frame[roi_high_top:roi_high_bot, 0:width])

    M_low  = cv2.moments(mask_low)
    M_high = cv2.moments(mask_high)

    # Affichage des lignes de guidage réajustées
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
                
                stable_dir = f"🚨 COUPE-FILE ULTRA-ANTICIPÉ ({int(angle_vector_deg)}°)"
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
                    stable_dir = "COMBO OVERRIDE : VERTICALE"
                    border_color = (255, 191, 0)
                else:
                    target_angle_delta = (angle_from_position * WEIGHT_POSITION) + (angle_from_direction * WEIGHT_DIRECTION)
                    stable_dir = "COMBO DICTION + POSITION"
        
        else:
            target_angle_delta = angle_base_low * 1.3
            stable_dir = "SUIVI SIMPLE BAS"
            bypass_smoothing = True
            
    elif pt_high is not None:
        line_seen = "OUI"
        error_high_px = pt_high[0] - center_x
        target_angle_delta = (error_high_px / center_x) * MAX_STEER_DELTA * 1.2
        stable_dir = "ACCROCHE SECU HAUT"

    # ── FILTRAGE ET CONSIGNES MATÉRIELLES ──
    if line_seen == "OUI":
        if bypass_smoothing:
            smoothed_angle_delta = target_angle_delta
        else:
            smoothed_angle_delta = (ALPHA_SMOOTHING * target_angle_delta) + ((1.0 - ALPHA_SMOOTHING) * smoothed_angle_delta)
        
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
        telemetry["line_seen"]   = line_seen
        telemetry["error_px"]    = int(final_angle_delta)
        telemetry["stable_dir"]  = stable_dir
        telemetry["distance_mm"] = current_dist
        telemetry["speed_pct"]   = calculated_speed
        telemetry["emergency"]   = is_emergency

    return output


# THREADS MATÉRIELS ET SENSEURS ASYNCHRONES
def thread_controller_camera_line(robot: Robot, interval: float) -> None:
    """Boucle matérielle principale d'actionnement de la propulsion et direction."""
    while True:
        with robot.state.lock:
            if not robot.state.running or not system_running:
                break
            emergency    = robot.state.emergency_stop
            target_speed = robot.state.calculated_speed
            target_angle = robot.state.calculated_angle

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
            robot.state.distance_mm    = dist_mm
            # Déclenchement arrêt d'urgence matériel
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
            angle     = robot.state.calculated_angle

        # Sélection de l'état lumineux selon les angles réels de braquage calculés
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


# STREAMING FLASK ET ACCÈS MATÉRIEL CAMERA
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
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Team C — Cockpit 2026</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#0d1117;
  --surface:#161b22;
  --card:#1c2128;
  --border:rgba(255,255,255,0.08);
  --border-em:rgba(255,255,255,0.15);
  --text:#e6edf3;
  --muted:#8b949e;
  --hint:#484f58;
  --mono:'SF Mono','Fira Code','Cascadia Code',monospace;
  --c-blue:#58a6ff;
  --c-blue-bg:rgba(88,166,255,0.1);
  --c-green:#3fb950;
  --c-green-bg:rgba(63,185,80,0.1);
  --c-red:#f85149;
  --c-red-bg:rgba(248,81,73,0.1);
  --c-amber:#d29922;
  --c-amber-bg:rgba(210,153,34,0.1);
  --c-purple:#bc8cff;
  --radius:8px;
  --radius-lg:12px;
}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;background:var(--bg);color:var(--text);padding:20px 24px;min-height:100vh}

/* ─ HEADER ─ */
header{display:flex;align-items:center;gap:12px;margin-bottom:20px;padding-bottom:16px;border-bottom:0.5px solid var(--border)}
.logo{width:34px;height:34px;border-radius:var(--radius);background:var(--c-blue-bg);border:0.5px solid var(--c-blue);display:flex;align-items:center;justify-content:center;flex-shrink:0}
.logo svg{width:18px;height:18px;stroke:var(--c-blue);fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
.header-text h1{font-size:15px;font-weight:600;color:var(--text);letter-spacing:0.3px}
.header-text p{font-size:12px;color:var(--muted);margin-top:2px}
#sys-badge{margin-left:auto;font-size:11px;font-weight:600;font-family:var(--mono);padding:4px 12px;border-radius:20px;letter-spacing:0.5px;text-transform:uppercase;transition:all .2s}
.badge-ok{background:var(--c-green-bg);color:var(--c-green);border:0.5px solid rgba(63,185,80,0.3)}
.badge-warn{background:var(--c-amber-bg);color:var(--c-amber);border:0.5px solid rgba(210,153,34,0.3)}
.badge-err{background:var(--c-red-bg);color:var(--c-red);border:0.5px solid rgba(248,81,73,0.3);animation:pulse 1s ease-in-out infinite alternate}
@keyframes pulse{from{box-shadow:none}to{box-shadow:0 0 12px rgba(248,81,73,0.35)}}

/* ─ LAYOUT ─ */
.layout{display:grid;grid-template-columns:1fr 290px;gap:16px;align-items:start;max-width:1060px}
@media(max-width:700px){.layout{grid-template-columns:1fr}}

/* ─ VIDEO ─ */
.video-card{border-radius:var(--radius-lg);overflow:hidden;border:0.5px solid var(--border);background:#000;position:relative}
.video-card img{display:block;width:100%}
.video-hud{position:absolute;top:10px;left:10px;right:10px;display:flex;justify-content:space-between;align-items:flex-start;pointer-events:none}
.hud-pill{font-family:var(--mono);font-size:11px;padding:3px 9px;border-radius:20px;background:rgba(0,0,0,0.55);backdrop-filter:blur(4px);color:#fff;border:0.5px solid rgba(255,255,255,0.12)}
.video-footer{padding:9px 13px;display:flex;align-items:center;gap:8px;border-top:0.5px solid var(--border);background:var(--card)}
.line-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0;transition:background .2s}
.line-dot.on{background:var(--c-green)}
.line-dot.off{background:var(--c-red)}
#line-status-text{font-size:12px;color:var(--muted);font-family:var(--mono)}
.strat-bar{position:absolute;bottom:0;left:0;right:0;padding:9px 13px;background:linear-gradient(transparent,rgba(0,0,0,0.75));font-family:var(--mono);font-size:12px;color:#fff;pointer-events:none}

/* ─ PANEL ─ */
.panel{display:flex;flex-direction:column;gap:12px}

.card{background:var(--card);border:0.5px solid var(--border);border-radius:var(--radius-lg);overflow:hidden}
.card-header{display:flex;align-items:center;gap:8px;padding:9px 13px;border-bottom:0.5px solid var(--border);background:var(--surface)}
.card-header .icon{width:20px;height:20px;display:flex;align-items:center;justify-content:center}
.card-header .icon svg{width:14px;height:14px;stroke:var(--muted);fill:none;stroke-width:1.8;stroke-linecap:round;stroke-linejoin:round}
.card-title{font-size:11px;font-weight:600;color:var(--muted);text-transform:uppercase;letter-spacing:0.7px}

.metric{display:flex;justify-content:space-between;align-items:center;padding:9px 13px;border-bottom:0.5px solid var(--border)}
.metric:last-child{border-bottom:none}
.metric-label{font-size:12px;color:var(--muted)}
.mval{font-family:var(--mono);font-size:13px;font-weight:600}
.mval.blue{color:var(--c-blue)}
.mval.green{color:var(--c-green)}
.mval.red{color:var(--c-red)}
.mval.amber{color:var(--c-amber)}
.mval.white{color:var(--text)}

/* ─ GAUGE ─ */
.gauge-wrap{padding:10px 13px}
.gauge-meta{display:flex;justify-content:space-between;font-size:12px;margin-bottom:6px}
.gauge-meta span:first-child{color:var(--muted)}
.gauge-meta span:last-child{font-family:var(--mono);font-weight:600;color:var(--text)}
.gauge-track{height:5px;border-radius:3px;background:rgba(255,255,255,0.06);overflow:hidden}
.gauge-fill{height:100%;border-radius:3px;transition:width .18s ease}
.gauge-fill.g-blue{background:var(--c-blue)}
.gauge-fill.g-green{background:var(--c-green)}
.gauge-fill.g-red{background:var(--c-red)}
.gauge-fill.g-amber{background:var(--c-amber)}

.steer-track{height:5px;border-radius:3px;background:rgba(255,255,255,0.06);position:relative;overflow:hidden}
.steer-center-line{position:absolute;left:50%;top:0;width:1px;height:100%;background:rgba(255,255,255,0.2)}
.steer-fill{height:100%;transition:all .18s ease;position:absolute}

/* ─ ESTOP ─ */
.estop-pill{display:inline-flex;align-items:center;gap:6px;font-size:11px;font-weight:600;font-family:var(--mono);padding:3px 10px;border-radius:20px;text-transform:uppercase;letter-spacing:0.5px}
.ep-ok{background:var(--c-green-bg);color:var(--c-green);border:0.5px solid rgba(63,185,80,0.3)}
.ep-ko{background:var(--c-red-bg);color:var(--c-red);border:0.5px solid rgba(248,81,73,0.3)}
.ep-dot{width:5px;height:5px;border-radius:50%}
.ep-dot.dok{background:var(--c-green)}
.ep-dot.dko{background:var(--c-red)}

/* ─ STRATEGY ─ */
.strat-card{background:var(--card);border:0.5px solid var(--border);border-radius:var(--radius-lg);padding:13px}
.strat-label{font-size:10px;font-weight:600;color:var(--hint);text-transform:uppercase;letter-spacing:1px;margin-bottom:8px}
#strat-value{font-family:var(--mono);font-size:13px;font-weight:600;line-height:1.5;transition:color .2s}

/* ─ SLOTS MODULAIRES SUPPLÉMENTAIRES ─ */
/* Ajoutez ici de nouveaux composants plus tard. */
/* Ex: .module-map, .module-imu, .module-log  */
</style>

<header>
  <div class="logo">
    <svg viewBox="0 0 24 24"><circle cx="12" cy="12" r="3"/><path d="M12 2v3M12 19v3M4.22 4.22l2.12 2.12M17.66 17.66l2.12 2.12M2 12h3M19 12h3M4.22 19.78l2.12-2.12M17.66 6.34l2.12-2.12"/></svg>
  </div>
  <div class="header-text">
    <h1>Cockpit de Télémétrie — Team C</h1>
    <p>Robot autonome &middot; suivi de ligne &middot; 2026</p>
  </div>
  <div id="sys-badge" class="badge-ok">Nominal</div>
</header>

<div class="layout">

  <!-- FLUX VIDÉO -->
  <div class="video-card">
    <img src="/video_feed" alt="Flux caméra du robot">
    <div class="video-hud">
      <div class="hud-pill" id="fps-hud">— Hz</div>
      <div class="hud-pill" id="dist-hud">— mm</div>
    </div>
    <div class="strat-bar" id="strat-hud">RECHERCHE...</div>
  </div>
  <div style="background:var(--card);border:0.5px solid var(--border);border-bottom-left-radius:var(--radius-lg);border-bottom-right-radius:var(--radius-lg);padding:9px 13px;display:flex;align-items:center;gap:8px;margin-top:-1px">
    <div class="line-dot off" id="line-dot"></div>
    <span id="line-status-text" style="font-size:12px;color:var(--muted);font-family:var(--mono)">Ligne non détectée</span>
  </div>

  <!-- PANNEAU DROITE -->
  <div class="panel">

    <!-- Cinématique -->
    <div class="card">
      <div class="card-header">
        <div class="icon">
          <svg viewBox="0 0 24 24"><path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"/></svg>
        </div>
        <span class="card-title">Cinématique</span>
      </div>
      <div class="gauge-wrap">
        <div class="gauge-meta"><span>Régime moteur</span><span id="speed-label">0 %</span></div>
        <div class="gauge-track"><div class="gauge-fill g-blue" id="speed-bar" style="width:0%"></div></div>
      </div>
      <div class="gauge-wrap" style="padding-top:0">
        <div class="gauge-meta"><span>Direction servo</span><span id="steer-label">0°</span></div>
        <div class="steer-track">
          <div class="steer-center-line"></div>
          <div class="steer-fill" id="steer-fill"></div>
        </div>
      </div>
    </div>

    <!-- Sécurité -->
    <div class="card">
      <div class="card-header">
        <div class="icon">
          <svg viewBox="0 0 24 24"><path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/></svg>
        </div>
        <span class="card-title">Sécurité</span>
      </div>
      <div class="metric">
        <span class="metric-label">Télémétrie ultrason</span>
        <span class="mval blue" id="dist-val">— mm</span>
      </div>
      <div class="metric">
        <span class="metric-label">Arrêt d'urgence</span>
        <div class="estop-pill ep-ok" id="estop">
          <div class="ep-dot dok" id="ep-dot"></div>
          <span id="estop-text">Nominal</span>
        </div>
      </div>
    </div>

    <!-- Vision -->
    <div class="card">
      <div class="card-header">
        <div class="icon">
          <svg viewBox="0 0 24 24"><path d="M1 12s4-8 11-8 11 8 11 8-4 8-11 8-11-8-11-8z"/><circle cx="12" cy="12" r="3"/></svg>
        </div>
        <span class="card-title">Vision</span>
      </div>
      <div class="metric">
        <span class="metric-label">Cadence d'images</span>
        <span class="mval white" id="fps-val">— Hz</span>
      </div>
      <div class="metric">
        <span class="metric-label">Correction angle</span>
        <span class="mval amber" id="err-val">0°</span>
      </div>
    </div>

    <!-- Stratégie -->
    <div class="strat-card">
      <div class="strat-label">Stratégie active</div>
      <div id="strat-value">RECHERCHE LIGNE...</div>
    </div>

    <!-- ZONE MODULAIRE — Ajoutez vos futurs composants ici -->
    <!-- <div id="slot-module-imu" class="card" style="display:none"></div>  -->
    <!-- <div id="slot-module-log" class="card" style="display:none"></div>  -->

  </div>
</div>

<script>
const SPEED_MAX = 40;
const STEER_CENTER = 90;
const MAX_DELTA = 45;

async function tick() {
  let d;
  try { d = await fetch('/data').then(r => r.json()); }
  catch(e) { return; }
  if (d.fps === undefined) return;

  // FPS
  document.getElementById('fps-val').textContent = d.fps + ' Hz';
  document.getElementById('fps-hud').textContent = d.fps + ' Hz';

  // Distance
  const dist = d.distance_mm;
  const distEl = document.getElementById('dist-val');
  distEl.textContent = dist + ' mm';
  distEl.className = 'mval ' + (dist < 150 ? 'red' : dist < 300 ? 'amber' : 'blue');
  document.getElementById('dist-hud').textContent = dist + ' mm';

  // Speed
  const spd = d.speed_pct;
  document.getElementById('speed-label').textContent = spd + ' %';
  document.getElementById('speed-bar').style.width = Math.round((spd / SPEED_MAX) * 100) + '%';
  const gc = spd > 35 ? 'g-green' : spd > 20 ? 'g-blue' : 'g-amber';
  document.getElementById('speed-bar').className = 'gauge-fill ' + gc;

  // Steer
  const delta = d.error_px;
  document.getElementById('steer-label').textContent = (delta >= 0 ? '+' : '') + delta + '°';
  document.getElementById('err-val').textContent = (delta >= 0 ? '+' : '') + delta + '°';
  const sf = document.getElementById('steer-fill');
  const pct = Math.abs(delta) / MAX_DELTA * 50;
  if (delta > 0) {
    sf.style.left = '50%'; sf.style.right = 'auto';
    sf.style.width = pct + '%';
    sf.style.background = 'var(--c-blue)';
  } else if (delta < 0) {
    sf.style.right = '50%'; sf.style.left = 'auto';
    sf.style.width = pct + '%';
    sf.style.background = 'var(--c-amber)';
  } else {
    sf.style.width = '0';
  }

  // Emergency stop
  const em = d.emergency;
  const estopEl = document.getElementById('estop');
  const dotEl = document.getElementById('ep-dot');
  const etxtEl = document.getElementById('estop-text');
  if (em) {
    estopEl.className = 'estop-pill ep-ko';
    dotEl.className = 'ep-dot dko';
    etxtEl.textContent = 'ARRÊT URGENCE';
  } else {
    estopEl.className = 'estop-pill ep-ok';
    dotEl.className = 'ep-dot dok';
    etxtEl.textContent = 'Nominal';
  }

  // Line detection
  const seen = d.line_seen === 'OUI';
  const dot = document.getElementById('line-dot');
  dot.className = 'line-dot ' + (seen ? 'on' : 'off');
  document.getElementById('line-status-text').textContent =
    seen ? ('Ligne détectée — ' + (delta > 3 ? 'correction droite' : delta < -3 ? 'correction gauche' : 'axe centré'))
         : 'Ligne non détectée';

  // Strategy
  const strat = d.stable_dir || '—';
  const sv = document.getElementById('strat-value');
  sv.textContent = strat;
  sv.style.color = em ? 'var(--c-red)'
    : strat.includes('URGENCE') ? 'var(--c-red)'
    : strat.includes('OVERRIDE') || strat.includes('COUPE') ? 'var(--c-amber)'
    : strat.includes('COMBO') ? 'var(--c-blue)'
    : 'var(--text)';
  document.getElementById('strat-hud').textContent = strat;

  // System badge
  const badge = document.getElementById('sys-badge');
  if (em) {
    badge.className = 'badge-err'; badge.textContent = 'URGENCE';
  } else if (!seen) {
    badge.className = 'badge-warn'; badge.textContent = 'Ligne perdue';
  } else {
    badge.className = 'badge-ok'; badge.textContent = 'Nominal';
  }
}

setInterval(tick, 80);
</script>
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


# POINT D'ENTRÉE PRINCIPAL D'EXÉCUTION
if __name__ == "__main__":
    args = parse_args()

    subprocess.run(["sudo", "pkill", "-f", "rpicam"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)

    robot = Robot(args)
    robot.init()
    
    # Orientation physique initiale de l'axe vertical caméra
    robot.head.set_angle_motor(2, 60)
    
    with robot.state.lock:
        robot.state.calculated_speed = 0
        robot.state.calculated_angle = STEER_CENTER_DEG
        robot.state.distance_mm    = 999
        robot.state.emergency_stop = False
        
    global_robot_ref = robot 

    # Démarrage synchrone de tous les threads
    threads = [
        threading.Thread(target=thread_controller_camera_line, args=(robot, CTRL_INTERVAL), name="CTRL", daemon=True),
        threading.Thread(target=thread_ultrasonic, args=(robot, US_INTERVAL), name="US", daemon=True),
        threading.Thread(target=thread_LED, args=(robot, LED_INTERVAL), name="LED", daemon=True),
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