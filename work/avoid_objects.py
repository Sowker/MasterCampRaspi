# ═══════════════════════════════════════════════════════════════════
#  IMPORTS
# ═══════════════════════════════════════════════════════════════════
# imports basic python
import time
import threading
import logging
# imports for rpi
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

# imports from helpers and tasks
import logger
from t11_argument_parser import parse_args
from t11_robot import Robot
from avoid_objects_threads import thread_ultrasonic_scanning, thread_object_controller, thread_line_detect_avoid, thread_avoid_line_controller

# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    log = logger.get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║  Robot Line Follower — Team C — SE 2026      ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    threads = [
        threading.Thread(target=thread_ultrasonic_scanning, args=(robot, args.sensor_interval), name="US", daemon=True),
        threading.Thread(target=thread_object_controller, args=(robot, args.ctrl_interval), name="CTRL_OBJ", daemon=True),
        threading.Thread(target=thread_line_detect_avoid, args=(robot, args.sensor_interval), name="LINE", daemon=True),
        threading.Thread(target=thread_avoid_line_controller, args=(robot, args.ctrl_interval), name="CTRL_LINE", daemon=True),
    ]

    for t in threads:
        t.start()
        log.info("Thread %-4s démarré (ident: %s)", t.name, t.ident)

    log.info("Tous les threads actifs. Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            time.sleep(0.5)

    except KeyboardInterrupt:
        log.info("Ctrl+C reçu — arrêt en cours…")

    finally:
        with robot.state.lock:
            robot.state.running = False

        for t in threads:
            t.join(timeout=4.0)
            if t.is_alive():
                log.warning("Thread %s ne s'est pas arrêté dans le délai", t.name)

        robot.shutdown()

    log.info("Programme terminé. Au revoir !")
    log.info("Program developed by Team C — MasterCamp SE 2026.")
