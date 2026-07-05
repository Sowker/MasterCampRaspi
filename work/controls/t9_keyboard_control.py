"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Keyboard controlled — Main Controller                ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture épurée :                                             ║
║    • Saisie et exécution directes via input() standard             ║
║    • Thread de sécurité pour l'ultrason + Feux de détresse (20cm)  ║
╚════════════════════════════════════════════════════════════════════╝
"""

import time
import argparse
import threading
from dataclasses import dataclass, field

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import logging
import logging.logger

from components.t1_front_led import FrontLEDs
from components.t2_back_led import Adeept_SPI_LedPixel
from components.t3_servomotors import Head
from components.t4_dc_motor import DCMotor, Direction, SPEED_NORMAL_PCT
from components.t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER

# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

PCA_ADDRESS = 0x5F
PCA_FREQUENCY_HZ = 50
OBSTACLE_THRESHOLD_MM = 200.0  # Seuil fixé à 20 cm pour les feux de détresse


# ═══════════════════════════════════════════════════════════════════
#  ÉTAT PARTAGÉ (Thread-Safe)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = True


# ═══════════════════════════════════════════════════════════════════
#  ROBOT — CLASSE DE HAUT NIVEAU
# ═══════════════════════════════════════════════════════════════════

class Robot:
    def __init__(self, cfg: argparse.Namespace):
        self._log = logger.get_logger("ROBOT")
        self._cfg = cfg

        self._log.info("Initialisation du bus I²C et PCA9685 (addr=0x%02X)…", PCA_ADDRESS)
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = PCA_FREQUENCY_HZ

        self.ultrasonic = UltrasonicSensor(cfg.us_trigger, cfg.us_echo)
        self.motor = DCMotor(self._pca)
        self.head = Head(self._pca)

        self.led = Adeept_SPI_LedPixel(14, 255)
        
        # Initialisation des LEDs avant
        self._log.info("Initialisation des LEDs avant…")
        self.front_leds = FrontLEDs()
        self.front_leds.start()

        self.state = RobotState()

    def init(self) -> None:
        self._log.info("══ Mise à zéro initiale ══")
        self.motor.reset()
        self.head.reset()
        time.sleep(0.5)
        if self.led.check_spi_state() != 0:
            self.led.start()
        self._log.info("Robot prêt.")

    def shutdown(self) -> None:
        self._log.info("══ Shutdown — remise à zéro ══")
        self.motor.reset()
        self.head.shutdown()
        self.front_leds.stop()  # Éteindre les LEDs avant
        if self.led.is_alive():
            self.led.stop()
            self.led.join(timeout=2.0)
        self.led.led_close()
        time.sleep(0.5)
        self._pca.deinit()
        self._log.info("PCA9685 désactivé. Bonne journée !")


# ═══════════════════════════════════════════════════════════════════
#  THREAD SÉCURITÉ ET FEUX DE DÉTRESSE
# ═══════════════════════════════════════════════════════════════════

def thread_security_and_led(robot: Robot, interval: float, threshold_mm: float):
    """
    Surveille la distance en arrière-plan. Si un objet est détecté à moins de 20cm :
    - Déclenche les feux de détresse (warning).
    - Arrête les moteurs par sécurité.
    """
    log = logger.get_logger("SECURITY")
    log.info("Thread Sécurité/LED démarré (intervalle=%.3f s)", interval)

    warning_actif = False

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        # Lecture de la distance réelle du capteur
        distance = robot.ultrasonic.read_mm()

        if distance <= threshold_mm:
            # Sécurité active : Objet à moins de 20cm
            robot.motor.stop()

            if not warning_actif:
                log.warning("⚠ OBSTACLE PROCHE (%.1f mm) ! Activation des feux de détresse.", distance)
                if hasattr(robot, 'led'):
                    robot.led.warning()
                    robot.front_leds.set_blink('warning')  # Synchroniser les LEDs avant
                warning_actif = True
        else:
            # Zone sûre
            if warning_actif:
                log.info("Obstacle écarté. Arrêt des feux de détresse.")
                if hasattr(robot, 'led'):
                    robot.led.arreter_warning()
                    robot.front_leds.cancel_blink()  # Arrêter les LEDs avant
                warning_actif = False

        time.sleep(interval)

    # Extinction propre des warnings à l'arrêt du thread
    if hasattr(robot, 'led'):
        robot.led.arreter_warning()
    robot.front_leds.cancel_blink()  # Arrêter les LEDs avant
    log.info("Thread Sécurité/LED arrêté")


# ═══════════════════════════════════════════════════════════════════
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot Keyboard controlled — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger", type=int, default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo", type=int, default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm", type=float, default=OBSTACLE_THRESHOLD_MM)

    p.add_argument("--debug", action="store_true", help="Active les logs DEBUG")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE (Saisie Principale)
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    log = logger.get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║ Robot Keyboard Controlled — Team C — SE 2026 ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    # Démarrage du thread de surveillance de l'ultrason + LED (période de 50ms)
    t_secu = threading.Thread(
        target=thread_security_and_led,
        args=(robot, 0.05, args.obstacle_mm),
        name="SECURITY",
        daemon=True
    )
    t_secu.start()

    print("\n=== GUIDE DES COMMANDES ===")
    print("z : Marche AVANT")
    print("s : Marche ARRIÈRE")
    print("a : ARRÊT Moteurs")
    print("exit : Quitter le programme\n")
    print("👉 Entrez votre commande puis validez avec ENTREE.\n")

    try:
        while True:
            cmd = input("Commande > ").strip().lower()

            if cmd == "exit":
                break

            if cmd == 'z':
                log.info("Action : En avant")
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            elif cmd == 's':
                log.info("Action : En arrière")
                robot.motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT)
            elif cmd == 'a':
                log.info("Action : Arrêt")
                robot.motor.stop()
            elif cmd != "":
                print("Commande invalide. Utilisez uniquement : z, s, a ou exit")

    except KeyboardInterrupt:
        log.info("Arrêt demandé par Ctrl+C…")

    finally:
        # Indique au thread de s'arrêter
        with robot.state.lock:
            robot.state.running = False

        t_secu.join(timeout=1.0)
        robot.shutdown()
        log.info("Programme terminé. Au revoir !")