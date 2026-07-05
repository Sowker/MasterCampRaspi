"""
╔════════════════════════════════════════════════════════════════════╗
║         Robot Line Follower — Main Controller                      ║
║         Team C — MasterCamp SE 2026                                ║
╠════════════════════════════════════════════════════════════════════╣
║  Architecture multithreads :                                       ║
║    • Thread US    — Capteur ultrason (obstacle)                    ║
║    • Thread LINE  — Capteurs de ligne infrarouges avec read_action ║
║    • Thread CTRL  — Décision + commande moteur/direction           ║
║    • Thread MAIN  — Démarrage, supervision, arrêt propre           ║
╚════════════════════════════════════════════════════════════════════╝
"""

# ═══════════════════════════════════════════════════════════════════
#  IMPORTS
# ═══════════════════════════════════════════════════════════════════

import time
import threading
import argparse
from dataclasses import dataclass, field
from typing import Optional

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

import logging
import logger

from components.t3_servomotors import Head, STEER_HARD_DEG, STEER_SOFT_DEG
from components.t4_dc_motor import DCMotor, Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from components.t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER
from components.t6_line_tracking import LineTracker, LinePosition, PIN_LINE_LEFT, PIN_LINE_MIDDLE, PIN_LINE_RIGHT

from components.t2_back_led import Adeept_SPI_LedPixel

# ═══════════════════════════════════════════════════════════════════
#  CONSTANTES DE CONFIGURATION
# ═══════════════════════════════════════════════════════════════════

# ── PCA9685 ────────────────────────────────────────────────────────
PCA_ADDRESS        = 0x5F
PCA_FREQUENCY_HZ   = 50

# ── Contrôleur ─────────────────────────────────────────────────────
OBSTACLE_THRESHOLD_MM = 150.0  # mm — seuil d'arrêt d'urgence

CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs

# ═══════════════════════════════════════════════════════════════════
#  ÉTAT PARTAGÉ (thread-safe)
# ═══════════════════════════════════════════════════════════════════

@dataclass
class RobotState:
    """
    Source de vérité unique entre tous les threads.
    Chaque accès en lecture/écriture doit être protégé par self.lock.
    """
    lock: threading.Lock = field(default_factory=threading.Lock)

    # ── Données capteurs synthétisées ──────────────────────────────
    distance_mm: float = 9999.0
    line_action: LinePosition = LinePosition.LINE_LOST  # Stockage direct de l'action décodée

    # ── Commandes de supervision ──────────────────────────────────
    running:        bool = True    # False → tous les threads s'arrêtent
    emergency_stop: bool = False   # True  → obstacle détecté


# ═══════════════════════════════════════════════════════════════════
#  ROBOT — CLASSE DE HAUT NIVEAU
# ═══════════════════════════════════════════════════════════════════

class Robot:
    """Façade unique rassemblant tous les composants matériels."""

    def __init__(self, cfg: argparse.Namespace):
        self._log = logger.get_logger("ROBOT")
        self._cfg = cfg

        self._log.info("Initialisation du bus I²C et PCA9685 (addr=0x%02X)…", PCA_ADDRESS)
        self._i2c = busio.I2C(SCL, SDA)
        self._pca = PCA9685(self._i2c, address=PCA_ADDRESS)
        self._pca.frequency = PCA_FREQUENCY_HZ

        self.ultrasonic   = UltrasonicSensor(cfg.us_trigger, cfg.us_echo)
        self.line_tracker = LineTracker(cfg.line_left, cfg.line_mid, cfg.line_right)
        self.motor        = DCMotor(self._pca)
        self.head         = Head(self._pca)

        self.state = RobotState()
        self._obstacle_threshold_mm = cfg.obstacle_mm

        self.led = Adeept_SPI_LedPixel(14, 255)

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
        time.sleep(0.5)
        if self.led.is_alive():
            self.led.stop()
            self.led.join(timeout=2.0)
        self.led.led_close()
        self._pca.deinit()
        self._log.info("PCA9685 désactivé. Bonne journée !")


# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle et met à jour RobotState."""
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        dist_mm = robot.ultrasonic.read_mm()

        with robot.state.lock:
            robot.state.distance_mm    = dist_mm
            robot.state.emergency_stop = dist_mm < robot._obstacle_threshold_mm

        time.sleep(interval)

    log.info("Thread arrêté")


def thread_line(robot: Robot, interval: float) -> None:
    """
    Lit l'action décodée des capteurs en boucle (via read_action)
    et met à jour directement l'action sur le RobotState.
    """
    log = logger.get_logger("LINE")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        # Capture matérielle et décodage atomique (Hors du Lock pour optimiser)
        current_action = robot.line_tracker.read_action()

        with robot.state.lock:
            robot.state.line_action = current_action

        time.sleep(interval)

    log.info("Thread arrêté")

def thread_LED(robot: Robot, interval: float):
    log = logger.get_logger("LED")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    action = None
    while True:
        with robot.state.lock:

            if not robot.state.running:
                break
            action = robot.state.line_action
            if action == LinePosition.TURN_LEFT_SOFT or action == LinePosition.TURN_LEFT_HARD:
                robot.led.clignotant_gauche()
            elif action == LinePosition.TURN_RIGHT_SOFT or action == LinePosition.TURN_RIGHT_HARD:
                robot.led.clignotant_droit()
            elif action == LinePosition.LINE_LOST:
                robot.led.warning()
            elif action == LinePosition.STRAIGHT:
                robot.led.arreter_clignotants()
                robot.led.arreter_warning()


        time.sleep(interval)

    log.info("Thread arrêté")


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """
    log  = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LinePosition] = None

    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
            action    = robot.state.line_action

        # ── Arrêt d'urgence obstacle (Priorité 1) ─────────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
            time.sleep(interval)
            continue

        # ── Suivi de ligne décodé (Priorité 2) ────────────────────
        if action != last_action:
            log.info("Changement de comportement → %s", action.name)
            last_action = action

        if action == LinePosition.STRAIGHT:
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)

        elif action == LinePosition.TURN_LEFT_SOFT:
            robot.head.steer_left(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT)

        elif action == LinePosition.TURN_RIGHT_SOFT:
            robot.head.steer_right(STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT)

        elif action == LinePosition.TURN_LEFT_HARD:
            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD)

        elif action == LinePosition.TURN_RIGHT_HARD:
            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD)

        elif action == LinePosition.INTERSECTION:
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            log.info("Intersection détectée — passage tout droit")

        else:  # LinePosition.LINE_LOST
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("Ligne perdue — recherche active / attente…")

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


# ═══════════════════════════════════════════════════════════════════
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot Line Follower — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger",   type=int,   default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo",      type=int,   default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm",  type=float, default=OBSTACLE_THRESHOLD_MM)

    g_line = p.add_argument_group("Capteurs de ligne")
    g_line.add_argument("--line-left",  type=int, default=PIN_LINE_LEFT)
    g_line.add_argument("--line-mid",   type=int, default=PIN_LINE_MIDDLE)
    g_line.add_argument("--line-right", type=int, default=PIN_LINE_RIGHT)

    g_timing = p.add_argument_group("Timing")
    g_timing.add_argument("--ctrl-interval",   type=float, default=CTRL_INTERVAL_S)
    g_timing.add_argument("--sensor-interval", type=float, default=SENSOR_INTERVAL_S)

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
    log.info("║  Robot Line Follower — Team C — SE 2026      ║")
    log.info("╚══════════════════════════════════════════════╝")

    robot = Robot(args)
    robot.init()

    threads = [
        threading.Thread(target=thread_ultrasonic, args=(robot, args.sensor_interval), name="US", daemon=True),
        threading.Thread(target=thread_line, args=(robot, args.sensor_interval), name="LINE", daemon=True),
        threading.Thread(target=thread_LED, args=(robot, args.sensor_interval), name="LED", daemon=True),
        threading.Thread(target=thread_controller, args=(robot, args.ctrl_interval), name="CTRL", daemon=True),
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
            t.join(timeout=3.0)
            if t.is_alive():
                log.warning("Thread %s ne s'est pas arrêté dans le délai", t.name)

        robot.shutdown()

    log.info("Programme terminé. Au revoir !")
    log.info("Program developed by Team C — MasterCamp SE 2026.")
