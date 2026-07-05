import cv2
import numpy as np
import time


def apply_mask(hsv_image, lower_bound, upper_bound):
    mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def color_score(hsv_image, ranges):
    score = 0
    for lower_bound, upper_bound in ranges:
        score += cv2.countNonZero(apply_mask(hsv_image, lower_bound, upper_bound))
    return score


def frame_to_hsv(frame, source):
    if source == "rgb":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    return cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)


def classify_sign(hsv_image):
    blue_ranges = [((86, 80, 60), (125, 255, 255))]
    work_ranges = [
        ((0, 120, 70), (10, 255, 255)),
        ((170, 120, 70), (179, 255, 255)),
        ((11, 120, 70), (24, 255, 255)),
    ]

    blue_score = color_score(hsv_image, blue_ranges)
    work_score = color_score(hsv_image, work_ranges)

    min_score = max(500, int(hsv_image.shape[0] * hsv_image.shape[1] * 0.003))
    if blue_score < min_score and work_score < min_score:
        return "Aucun panneau detecte"

    if blue_score > work_score:
        return "Tunnel"
    if work_score > blue_score:
        return "Travaux"
    return "Aucun panneau detecte"


def add_noise(frame, sigma=10):
    noise = np.random.normal(0, sigma, frame.shape).astype(np.int16)
    noisy_frame = frame.astype(np.int16) + noise
    return np.clip(noisy_frame, 0, 255).astype(np.uint8)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Détection de couleurs de l'arc-en-ciel")
    parser.add_argument("--headless", action="store_true", help="Mode terminal uniquement")
    parser.add_argument("--power-pin", type=int, default=17, help="GPIO BCM pin pour alimenter la caméra (gpiozero OutputDevice)")
    args = parser.parse_args()

    POWER_PIN = args.power_pin

    cam_power = None
    picam = None
    camera = None

    # Detect availability of Picamera2 and gpiozero
    use_picamera2 = False
    have_gpio = False
    try:
        from picamera2 import Picamera2
        use_picamera2 = True
    except Exception:
        use_picamera2 = False

    try:
        from gpiozero import OutputDevice
        have_gpio = True
    except Exception:
        have_gpio = False

    try:
        if use_picamera2:
            if have_gpio:
                cam_power = OutputDevice(POWER_PIN, active_high=True, initial_value=False)
                print(f"Activation de la caméra via GPIO {POWER_PIN}")
                cam_power.on()
                time.sleep(1.0)  # laisser le temps à l'alimentation de se stabiliser

            picam = Picamera2()
            config = picam.create_video_configuration(main={"size": (640, 480)})
            picam.configure(config)
            picam.start()
            time.sleep(0.5)  # attente après l'initialisation

            label_prev = None
            while True:
                try:
                    frame = picam.capture_array()
                except RuntimeError as e:
                    print(f"Failed to capture frame: {e}")
                    break

                hsv_frame = frame_to_hsv(frame, "rgb")
                label = classify_sign(hsv_frame)

                if label != label_prev:
                    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {label}")
                    label_prev = label
        else:
            # Fallback vers webcam OpenCV (PC ou USB webcam)
            camera = cv2.VideoCapture(0)
            if not camera.isOpened():
                raise RuntimeError("Impossible d'ouvrir la camera.")

            label_prev = None
            while True:
                success, frame = camera.read()
                if not success:
                    raise RuntimeError("Impossible de lire une image depuis la camera.")

                hsv_frame = frame_to_hsv(frame, "bgr")
                label = classify_sign(hsv_frame)

                if label != label_prev:
                    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} - {label}")
                    label_prev = label

    except KeyboardInterrupt:
        print("Arrêt demandé — nettoyage...")
    finally:
        try:
            if picam is not None:
                picam.stop()
        except Exception:
            pass
        try:
            if camera is not None:
                camera.release()
        except Exception:
            pass
        try:
            if cam_power is not None:
                cam_power.off()
        except Exception:
            pass
