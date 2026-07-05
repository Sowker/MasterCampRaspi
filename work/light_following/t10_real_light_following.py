"""
╔════════════════════════════════════════════════════════════════════╗
║   Robot — Suivi de source lumineuse + obstacle (Tâche 10)          ║
║   Team C — MasterCamp SE 2026                                       ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture :                                                     ║
║    • Thread clavier   : 'M'/'m' = depart, 'A'/'a' = arret, exit     ║
║    • Boucle principale : suivi de lumiere (braquage roue avant)     ║
║                          + securite ultrason (feux de detresse,     ║
║                          recul + Bip Bip)                           ║
╚════════════════════════════════════════════════════════════════════╝
"""

import time
import argparse
import threading
import importlib
from dataclasses import dataclass, field

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import logging
import logger

from line_following.t11_buzzer_Sirene import POLICE,MII, play, close_buzzer
from components.t1_front_led import FrontLEDs
from components.t2_back_led import Adeept_SPI_LedPixel
from components.t3_servomotors import Head
from components.t4_dc_motor import DCMotor, Direction, SPEED_NORMAL_PCT, SPEED_BACKWARD
from components.t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER
from t8_light_following2 import LightFollowingModule


# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

PCA_ADDRESS = 0x5F
PCA_FREQUENCY_HZ = 50
OBSTACLE_THRESHOLD_MM = 200.0  # Seuil fixé à 20 cm

LIGHT_CH_LEFT  = 0
LIGHT_CH_RIGHT = 1

PAUSE_AVANT_RECUL_S  = 1.0  # attente avant le recul (point 4)
RECUL_DUREE_S        = 1.5  # ~30 cm a vitesse reduite (SPEED_BACKWARD)
PAUSE_APRES_RECUL_S  = 2.0  # arret avant reprise du suivi (point 5)

LOOP_PERIOD_S = 0.1


# ═══════════════════════════════════════════════════════════════════
#  ÉTAT PARTAGÉ (Thread-Safe)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = True
    driving: bool = False
    emergency_stop: bool = False


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
        self.light = LightFollowingModule(ch_left=cfg.light_left, ch_right=cfg.light_right)

        self.led = Adeept_SPI_LedPixel(14, 255)

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
        self.front_leds.stop()
        if self.led.is_alive():
            self.led.stop()
            self.led.join(timeout=2.0)
        self.led.led_close()
        close_buzzer()
        time.sleep(0.5)
        self._pca.deinit()
        self._log.info("PCA9685 désactivé. Bonne journée !")

    def feux_detresse(self, on: bool) -> None:
        if on:
            self.led.warning()
            self.front_leds.set_blink('warning')
        else:
            self.led.arreter_warning()
            self.front_leds.cancel_blink()


# ═══════════════════════════════════════════════════════════════════
#  GESTION OBSTACLE (Feux de détresse + recul + Bip Bip)
# ═══════════════════════════════════════════════════════════════════

def handle_obstacle(robot: Robot, log: logging.Logger) -> None:
    log.warning("⚠ OBSTACLE PROCHE ! Arrêt + feux de détresse.")
    robot.motor.stop()
    robot.feux_detresse(True)

    time.sleep(PAUSE_AVANT_RECUL_S)

    log.info("Recul ~30 cm à vitesse réduite + Bip Bip (sirène)")
    robot.motor.drive(Direction.BACKWARD, SPEED_BACKWARD, slow=True)
    time.sleep(RECUL_DUREE_S)
    robot.motor.stop()

    # Fin du recul -> on coupe la sirène POLICE ici
    with robot.state.lock:
        robot.state.emergency_stop = False

    robot.feux_detresse(False)

    log.info("Pause de 2 s avant reprise du suivi de lumière")
    time.sleep(PAUSE_APRES_RECUL_S)


# ═══════════════════════════════════════════════════════════════════
#  BOUCLE PRINCIPALE — SUIVI DE LUMIERE + SECURITE
# ═══════════════════════════════════════════════════════════════════

def light_following_loop(robot: Robot, threshold_mm: float) -> None:
    log = logger.get_logger("LIGHT")
    en_marche = False

    while True:
        with robot.state.lock:
            running = robot.state.running
            driving = robot.state.driving

        if not running:
            break

        if driving:
            if not en_marche:
                log.info("Départ — suivi de lumière activé")
                robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
                en_marche = True

            try:
                distance = robot.ultrasonic.read_mm()
            except OSError as e:
                log.error("[Ultrason] capteur inaccessible : %s", e)
                distance = None

            if distance is not None and distance <= threshold_mm:
                with robot.state.lock:
                    robot.state.emergency_stop = True

                handle_obstacle(robot, log)

                with robot.state.lock:
                    driving = robot.state.driving

                if driving:
                    log.info("Reprise du suivi de lumière")
                    robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
                else:
                    en_marche = False
                continue

            try:
                reading = robot.light.read()
                angle = robot.light.get_steer_angle(reading)
                robot.head.wheel.set_angle(angle)
            except OSError as e:
                log.error("[I2C] ADS7830 inaccessible (0x48) : %s", e)

        else:
            if en_marche:
                log.info("Arrêt demandé")
                robot.motor.stop()
                robot.head.wheel.center()
                en_marche = False

        time.sleep(LOOP_PERIOD_S)

    robot.motor.stop()
    log.info("Boucle suivi de lumière arrêtée")


# ═══════════════════════════════════════════════════════════════════
#  THREAD CLAVIER
# ═══════════════════════════════════════════════════════════════════

def keyboard_loop(robot: Robot) -> None:
    log = logger.get_logger("KEYBOARD")

    print("\n=== GUIDE DES COMMANDES ===")
    print("M ou m : Départ — suivi de lumière")
    print("A ou a : ARRÊT")
    print("exit   : Quitter le programme\n")
    print("👉 Entrez votre commande puis validez avec ENTREE.\n")

    while True:
        try:
            cmd = input("Commande > ").strip()
        except EOFError:
            cmd = "exit"

        if cmd.lower() == "exit":
            log.info("Commande reçue : exit")
            with robot.state.lock:
                robot.state.running = False
                robot.state.driving = False
            break

        if cmd in ("M", "m"):
            log.info("Commande reçue : M -> départ")
            with robot.state.lock:
                robot.state.driving = True
        elif cmd in ("A", "a"):
            log.info("Commande reçue : A -> arrêt")
            with robot.state.lock:
                robot.state.driving = False
        elif cmd != "":
            print("Commande invalide. Utilisez : M (marche), A/a (arrêt) ou exit")

def buzzer_loop(robot: Robot) -> None:
    log = logger.get_logger("BUZZER")
    log.info("Thread démarré")

    def emergency_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.emergency_stop

    def driving_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.driving and not robot.state.emergency_stop

    while True:
        with robot.state.lock:
            running   = robot.state.running
            emergency = robot.state.emergency_stop
            driving   = robot.state.driving

        if not running:
            break

        if emergency:
            play(POLICE, emergency_active)
        elif driving:
            play(MII, driving_active)
        else:
            time.sleep(LOOP_PERIOD_S)

    log.info("Thread arrêté")


# ═══════════════════════════════════════════════════════════════════
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot — Suivi de source lumineuse + obstacle — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger", type=int, default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo", type=int, default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm", type=float, default=OBSTACLE_THRESHOLD_MM)

    g_light = p.add_argument_group("Capteurs de lumière (ADS7830)")
    g_light.add_argument("--light-left", type=int, default=LIGHT_CH_LEFT)
    g_light.add_argument("--light-right", type=int, default=LIGHT_CH_RIGHT)

    p.add_argument("--debug", action="store_true", help="Active les logs DEBUG")
    return p.parse_args()


# ═══════════════════════════════════════════════════════════════════
#  POINT D'ENTRÉE
# ═══════════════════════════════════════════════════════════════════

if __name__ == "__main__":
    args = parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    log = logger.get_logger("MAIN")
    log.info("╔══════════════════════════════════════════════╗")
    log.info("║ Robot Suivi Lumière + Obstacle — Team C — SE 2026 ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    t_keyboard = threading.Thread(
        target=keyboard_loop,
        args=(robot,),
        name="KEYBOARD",
        daemon=True
    )
    t_keyboard.start()

    t_buzzer = threading.Thread(
        target=buzzer_loop,
        args=(robot,),
        name="BUZZER",
        daemon=True
    )
    t_buzzer.start()

    try:
        light_following_loop(robot, args.obstacle_mm)
    except KeyboardInterrupt:
        log.info("Arrêt demandé par Ctrl+C…")

    finally:
        with robot.state.lock:
            robot.state.running = False
            robot.state.driving = False
        t_buzzer.join(3.0)
        t_keyboard.join(3.0)
        robot.shutdown()
        log.info("Programme terminé. Au revoir !")
