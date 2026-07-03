import cv2
import numpy as np
import time
from picamera2 import Picamera2
from pannel_test import get_color_mask

picam = Picamera2()
picam.configure(picam.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)}))
picam.start()

last_print_t = 0
last_panneau = None

print("Démarrage — Ctrl+C pour arrêter. Images dans /tmp/pannel_*.jpg")

try:
    while True:
        frame = picam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        h, w  = frame.shape[:2]
        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        masks = get_color_mask(hsv)

        panneau_detecte = None

        for couleur, mask in masks.items():
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [c for c in contours if cv2.contourArea(c) > 500]
            if not contours:
                continue

            c     = max(contours, key=cv2.contourArea)
            ratio = cv2.contourArea(c) / (h * w)

            epsilon = 0.04 * cv2.arcLength(c, True)
            approx  = cv2.approxPolyDP(c, epsilon, True)
            n       = len(approx)

            forme = "triangle" if n == 3 else "rectangle" if n == 4 else f"{n} sommets"

            if   couleur == "bleu"  and n == 4: panneau = "TUNNEL"
            elif couleur == "jaune" and n == 3: panneau = "TRAVAUX"
            else:                               panneau = None

            if panneau:
                panneau_detecte = panneau

            print(f"  {couleur:6s} | {forme:12s} | {ratio*100:.1f}% | {panneau or '---'}", end="\r")

        # Affiche dans la console seulement si ça change
        now = time.monotonic()
        if panneau_detecte != last_panneau or now - last_print_t > 2.0:
            print(f"\n→ PANNEAU : {panneau_detecte or 'rien'}")
            last_panneau = panneau_detecte
            last_print_t = now

        # Sauvegarde images pour visualiser via scp
        cv2.imwrite("/tmp/pannel_frame.jpg", frame)
        cv2.imwrite("/tmp/pannel_bleu.jpg",  masks["bleu"])
        cv2.imwrite("/tmp/pannel_jaune.jpg", masks["jaune"])

        time.sleep(0.1)

except KeyboardInterrupt:
    pass

picam.stop()
print("\nArrêté.")
