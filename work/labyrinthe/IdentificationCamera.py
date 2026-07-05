import cv2
import numpy as np
from flask import Flask, render_template_string, Response, jsonify
import subprocess
import time
import threading
import collections
import signal
import sys
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger(__name__)

# ── Paramètres de filtrage morphologique stricts ──────────────────────────────
AREA_MIN        = 400       # Ignorer les micro-bruits de fond au loin
AREA_MAX        = 250000

RATIO_MIN       = 1.3       # Une flèche est nettement plus large que haute
RATIO_MAX       = 4.0

# SOLIDITÉ = Aire_Contour / Aire_Boite_Englobante
# Un rectangle ou un bloc de bruit a une solidité proche de 0.8 à 1.0.
# Une vraie flèche (avec le vide autour du fût) oscille entre 0.35 et 0.65.
SOLIDITY_MIN    = 0.32
SOLIDITY_MAX    = 0.68

# Voisinage pour l'analyse de l'épaisseur de la pointe
NEIGHBOR_PCT    = 0.22

# Stabilisation
VOTE_WINDOW     = 6
CONF_DECAY      = 0.65

lock              = threading.Lock()
direction_history = collections.deque(maxlen=VOTE_WINDOW)
confidence_smooth = 0.0

telemetry = {
    "camera_status": "Déconnectée",
    "arrows_count":  0,
    "direction":     "AUCUNE",
    "stable_dir":    "AUCUNE",
    "confidence":    0.0,
    "last_arrow_x":  "-",
    "last_arrow_y":  "-",
    "fps":           0,
}

app = Flask(__name__)



# ANALYSE MORPHOLOGIQUE ET SÉCURITÉ GÉOMÉTRIQUE
def validate_and_classify_arrow(cnt, approx):
    """
    Filtre drastiquement les formes parasites et extrait la direction réelle.
    """
    # Dimensions de la boîte englobante
    x, y, w, h = cv2.boundingRect(cnt)
    if h == 0 or w == 0:
        return "AUCUNE", 0.0, None, None

    # Filtre de Ratio Aspect
    ratio = w / h
    if not (RATIO_MIN <= ratio <= RATIO_MAX):
        return "AUCUNE", 0.0, None, None

    # FILTRE DE SOLIDITÉ (Anti-bruit & Anti-rectangle plein)
    area = cv2.contourArea(cnt)
    box_area = w * h
    solidity = area / box_area
    if not (SOLIDITY_MIN <= solidity <= SOLIDITY_MAX):
        return "AUCUNE", 0.0, None, None

    # Extraction des points extrêmes horizontaux
    flat_pts = approx.reshape(-1, 2)
    pt_left = flat_pts[np.argmin(flat_pts[:, 0])]
    pt_right = flat_pts[np.argmax(flat_pts[:, 0])]

    # Analyse de l'épaisseur locale (Où se trouve la pointe ?)
    # On regarde l'écartement vertical des points à gauche vs à droite
    left_neighbors = [p for p in flat_pts if abs(p[0] - pt_left[0]) < w * NEIGHBOR_PCT]
    right_neighbors = [p for p in flat_pts if abs(p[0] - pt_right[0]) < w * NEIGHBOR_PCT]

    left_spread = np.max([p[1] for p in left_neighbors]) - np.min([p[1] for p in left_neighbors]) if left_neighbors else h
    right_spread = np.max([p[1] for p in right_neighbors]) - np.min([p[1] for p in right_neighbors]) if right_neighbors else h

    # La pointe est le côté le plus fin/serré verticalement
    if right_spread < left_spread:
        direction = "DROITE"
        tip = pt_right
        base = pt_left
    else:
        direction = "GAUCHE"
        tip = pt_left
        base = pt_right

    # filtre de symétrie : La pointe doit être centrée horizontalement sur l'axe Y
    box_center_y = y + h / 2
    tip_center_error = abs(tip[1] - box_center_y) / h
    if tip_center_error > 0.22:  # Rejette si la pointe est excentrée (bruit asymétrique)
        return "AUCUNE", 0.0, None, None

    score = round(1.0 - tip_center_error, 2)
    return direction, score, tip, base


# PIPELINE DE TRAITEMENT D'IMAGE
def detect_and_orient_arrow(frame: np.ndarray) -> np.ndarray:
    global confidence_smooth, direction_history

    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur  = cv2.GaussianBlur(gray, (5, 5), 0)

    # Hausse des seuils pour supprimer les contours trop faibles à l'arrière-plan
    edges = cv2.Canny(blur, 70, 200)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    edges  = cv2.dilate(edges, kernel, iterations=1)

    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    output = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

    best_direction = "AUCUNE"
    best_score      = 0.0
    best_meta       = None
    arrows_found   = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (AREA_MIN <= area <= AREA_MAX):
            continue

        peri   = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)

        # Validation renforcée
        direction, score, tip, base = validate_and_classify_arrow(cnt, approx)
        if direction == "AUCUNE":
            continue

        arrows_found += 1
        if score > best_score:
            best_score     = score
            best_direction = direction
            color          = (0, 60, 255) if direction == "DROITE" else (255, 160, 0)
            best_meta      = (tip, base, approx, color)

    # ── Système de vote glissant ──────────────────────────────────────────────
    direction_history.append(best_direction)
    vote_counts = collections.Counter(direction_history)
    stable_dir  = vote_counts.most_common(1)[0][0]

    target_conf       = best_score if arrows_found > 0 else 0.0
    confidence_smooth = confidence_smooth * CONF_DECAY + target_conf * (1 - CONF_DECAY)
    confidence_smooth = round(min(1.0, confidence_smooth), 3)

    # ── Dessin des retours visuels ─────────────────────────────────────────────
    if best_meta:
        tip, base, approx, color = best_meta

        cv2.drawContours(output, [approx], -1, color, 2)
        cv2.circle(output, tuple(tip), 6, (255, 255, 0), -1)  # Tête en Cyan
        cv2.circle(output, tuple(base), 5, (0, 0, 255), 2)    # Base en Rouge
        cv2.arrowedLine(output, tuple(base), tuple(tip), color, 2, tipLength=0.15)

        x, y, w, h = cv2.boundingRect(approx)
        label = f"{'◀ ' if best_direction == 'GAUCHE' else ''}{best_direction}{' ▶' if best_direction == 'DROITE' else ''} [{best_score:.2f}]"
        cv2.putText(output, label, (x, max(y - 10, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

    # Affichage de l'état des votes en bas de l'image
    H, W = output.shape[:2]
    pct   = int(confidence_smooth * 200)
    bar_c = (0, 230, 100) if confidence_smooth > 0.5 else (0, 80, 255)
    cv2.rectangle(output, (10, H - 24), (210, H - 10), (30, 30, 30), -1)
    cv2.rectangle(output, (10, H - 24), (10 + pct, H - 10), bar_c, -1)

    votes_str = "".join("▶" if d == "DROITE" else "◀" if d == "GAUCHE" else "·" for d in direction_history)
    cv2.putText(output, f"Votes: [{votes_str}] -> {stable_dir}", (10, H - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

    with lock:
        telemetry["arrows_count"]   = arrows_found
        telemetry["direction"]      = best_direction
        telemetry["stable_dir"]     = stable_dir
        telemetry["confidence"]     = confidence_smooth
        if best_meta:
            telemetry["last_arrow_x"] = int(best_meta[0][0])
            telemetry["last_arrow_y"] = int(best_meta[0][1])
        else:
            telemetry["last_arrow_x"] = "-"
            telemetry["last_arrow_y"] = "-"

    return output


# ENTRÉE VIDÉO PICAM ET SERVEUR FLASK
def generate_frames():
    cmd = [
        "rpicam-vid", "-t", "0", "--inline",
        "--width", "640", "--height", "480",
        "--framerate", "30", "--codec", "mjpeg", "-o", "-"
    ]
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
    buf = b""

    with lock:
        telemetry["camera_status"] = "En ligne (rpicam)"

    frame_count = 0
    t0 = time.time()

    try:
        while True:
            chunk = process.stdout.read(4096)
            if not chunk: break
            buf += chunk

            a   = buf.find(b"\xff\xd8")
            b_e = buf.find(b"\xff\xd9")
            if a != -1 and b_e > a:
                jpg = buf[a: b_e + 2]
                buf = buf[b_e + 2:]

                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue

                frame_count += 1
                elapsed = time.time() - t0
                if elapsed >= 1.0:
                    with lock:
                        telemetry["fps"] = round(frame_count / elapsed, 1)
                    frame_count, t0 = 0, time.time()

                processed = detect_and_orient_arrow(frame)
                _, enc = cv2.imencode(".jpg", processed)
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + enc.tobytes() + b"\r\n")

    except Exception as e:
        log.error(f"Erreur flux : {e}")
    finally:
        with lock:
            telemetry["camera_status"] = "Hors ligne"
        process.terminate()


HTML = """<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Arrow Tracker V5</title>
<style>
:root{--bg:#070a12;--panel:#0e131f;--border:#1f293d;--text:#b2c0d6;--cyan:#00e5ff;--red:#ff3d57;--green:#00e676;--mono:monospace}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:sans-serif;background:var(--bg);color:var(--text);padding:20px}
header{text-align:center;margin-bottom:20px}
header h1{font-size:1.3rem;color:var(--green);letter-spacing:1px}
.layout{display:flex;flex-wrap:wrap;gap:20px;justify-content:center;max-width:1000px;margin:auto}
.vbox{position:relative;border:2px solid var(--border);border-radius:8px;overflow:hidden;width:640px;background:#000}
.vbox img{display:block;width:100%}
.panel{background:var(--panel);border:1px solid var(--border);border-radius:8px;padding:20px;width:320px;display:flex;flex-direction:column;gap:12px}
.ptitle{font-size:.65rem;text-transform:uppercase;color:#4f637d;border-bottom:1px solid var(--border);padding-bottom:5px}
.row{display:flex;justify-content:space-between;font-size:.85rem}
.val{font-family:var(--mono);font-weight:700;color:#fff}
.dir-main{text-align:center;font-size:2.8rem;font-weight:900;font-family:var(--mono);margin:10px 0}
</style>
<script>
async function tick(){
  const d=await fetch('/data').then(r=>r.json());
  document.getElementById('fps').textContent=d.fps+' Hz';
  document.getElementById('cnt').textContent=d.arrows_count;

  const el=document.getElementById('dir-val');
  el.textContent=d.stable_dir;
  el.style.color=d.stable_dir==='DROITE'?'#ff3d57':d.stable_dir==='GAUCHE'?'#00e5ff':'#4f637d';
  document.getElementById('vbox').style.borderColor=el.style.color;
}
setInterval(tick,100);
</script>
</head>
<body>
<header>
  <h1>🔺 Arrow Tracker V5 — Filtre Anti-Faux Positifs</h1>
  <p>Isolation par Solidité Morphologique & Symétrie Axiale</p>
</header>
<div class="layout">
  <div class="vbox" id="vbox"><img src="/video_feed"></div>
  <div class="panel">
    <div class="ptitle">Données Robot</div>
    <div class="row"><span>Fréquence</span><span class="val" id="fps">— Hz</span></div>
    <div class="row"><span>Flèches Validées</span><span class="val" id="cnt">0</span></div>
    <div class="ptitle">Direction Filtrée</div>
    <div class="dir-main" id="dir-val">AUCUNE</div>
    <div class="ptitle">Légende</div>
    <div style="font-size: .7rem; line-height: 1.4; color: #697c98;">
      • <span style="color:#00e5ff; font-weight:bold;">Point Cyan plein</span> : Pointe validée<br>
      • <span style="color:#ff3d57; font-weight:bold;">Cercle Rouge</span> : Extrémité opposée<br>
      • Solidité requise : 32% à 68% de la boîte
    </div>
  </div>
</div>
</body>
</html>"""

@app.route("/")
def index(): return HTML

@app.route("/video_feed")
def video_feed(): return Response(generate_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/data")
def get_data():
    with lock: return jsonify(telemetry)

if __name__ == "__main__":
    subprocess.run(["sudo", "pkill", "-f", "rpicam"], stderr=subprocess.DEVNULL)
    time.sleep(0.2)
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)