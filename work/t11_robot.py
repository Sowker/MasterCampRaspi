import time
import threading
import argparse
from dataclasses import dataclass, field

from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685



from t1_front_led import FrontLEDs
from t2_back_led import Adeept_SPI_LedPixel
from t3_servomotors import Head, STEER_HARD_DEG, STEER_SOFT_DEG
from t4_dc_motor import DCMotor, Direction, SPEED_BACKWARD, SPEED_TURNING_PCT, SPEED_NORMAL_PCT
from t5_ultrasonic_sensor import UltrasonicSensor, PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER
from t6_line_tracking import LineTracker, LinePosition, PIN_LINE_LEFT, PIN_LINE_MIDDLE, PIN_LINE_RIGHT
from t11_buzzer_Sirene import POLICE, MII, play, close_buzzer
import logger

# ── PCA9685 ────────────────────────────────────────────────────────
PCA_ADDRESS        = 0x5F
PCA_FREQUENCY_HZ   = 50


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
    driving:        bool = False   # True  → le robot avance
    maneuver:       bool = False   # True  → manœuvre de récupération (ligne perdue)
    lost_time:      float = None # -> is the time when we lost the line
    post_time:      float = None # -> time for fine tuning post manuver
    post_manuver:   bool = False   # TRUE -> do the post manuver tuning
    shutdown:       bool = False   # TRUE -> kill the robot
    already_lost:   bool = False   # True -> count until he is really lost
    last_turn:      str  = 0       # -1 : Gauche, 0 : Tout droit, 1 : Droite
    action:         str  = 0       # hold all the actions of the robot


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

        self._log.info("Initialisation des LEDs avant…")
        self.front_leds = FrontLEDs()

    def init(self) -> None:
        self._log.info("══ Mise à zéro initiale ══")
        self.motor.reset()
        self.head.reset()
        time.sleep(0.5)
        if self.led.check_spi_state() != 0:
            self.led.start()
        self.front_leds.start()
        self._log.info("Robot prêt.")

    def shutdown(self) -> None:
        self._log.info("══ Shutdown — remise à zéro ══")
        self.motor.reset()
        self.head.shutdown()
        self.front_leds.stop()
        time.sleep(0.5)
        if self.led.is_alive():
            self.led.stop()
            self.led.join(timeout=2.0)
        self.led.led_close()
        close_buzzer()
        self._pca.deinit()
        self._log.info("PCA9685 désactivé. Bonne journée !")