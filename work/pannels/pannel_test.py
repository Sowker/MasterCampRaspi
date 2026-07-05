import cv2
import numpy as np
from picamera2 import Picamera2 
import time
import logging.logger

def get_color_mask(hsv):
    masks = {}
    masks["bleu"]  = cv2.inRange(hsv, np.array([100, 100, 100]), np.array([130, 255, 255]))
    masks["jaune"] = cv2.inRange(hsv, np.array([20,  100, 100]), np.array([35,  255, 255]))
    return masks

def detect_sign(frame):
    h, w = frame.shape[:2]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    masks = get_color_mask(hsv)

    for couleur, mask in masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) > 500]
        if not contours:
            continue

        c = max(contours, key=cv2.contourArea)

        ratio = cv2.contourArea(c) / (h * w)
        if ratio < 0.03:
            continue

        epsilon = 0.04 * cv2.arcLength(c, True)
        approx  = cv2.approxPolyDP(c, epsilon, True)
        n       = len(approx)

        if couleur == "bleu"  and n == 4: return "tunnel"
        if couleur == "jaune" and n == 3: return "travaux"

    return None
