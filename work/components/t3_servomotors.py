import time
from adafruit_motor import servo as adafruit_servo
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

import logging.logger as log

# ── constantes ─────────────────────────────────────────────────────────
SERVO_MIN_PULSE_US  = 500                # µs — impulsion minimale
SERVO_MAX_PULSE_US  = 2400               # µs — impulsion maximale
SERVO_RANGE_DEG     = 180                # degrés d'actuation totale
SERVO_INIT_DELAY_S  = 0.3                # délai après positionnement initial
I2C = busio.I2C(SCL, SDA)
SERVO_PCA = PCA9685(I2C, address=0x5f)   # default address 0x40
SERVO_PCA.frequency = 50                 #


# Limites angulaires (mécaniques)
WHEEL_ANGLE_MIN     = 46    # degrés — braquage gauche max
WHEEL_ANGLE_CENTER  = 84     # degrés — tout droit
WHEEL_ANGLE_MAX     = 122    # degrés — braquage droite max

HEAD_ANGLE_MIN      = 13     # degrés
HEAD_ANGLE_CENTER   = 93     # degrés
HEAD_ANGLE_MAX      = 173    # degrés

# Pas et délai pour move_to()
SERVO_STEP_DEG      = 0.5    # degrés par pas (flottant)
SERVO_STEP_DELAY_S  = 0.008  # secondes entre chaque pas

CHANNEL_SERVO_WHEEL      = 0   # Roue directionnelle
CHANNEL_SERVO_HORIZONTAL = 1   # Tête pan (gauche/droite)
CHANNEL_SERVO_VERTICAL   = 2   # Tête tilt (haut/bas)

STEER_SOFT_DEG        = 15   # degrés de braquage — correction légère
STEER_HARD_DEG        = 35   # degrés de braquage — virage serré


#  HARDWARE — SERVO-MOTEUR
class ServoMotor:
    """Contrôle d'un servo PCA9685 avec suivi interne de l'angle (float)."""

    def __init__(self,
                 pca_instance:   PCA9685,
                 channel:        int,
                 angle_min:      float = 10.0,
                 angle_max:      float = 170.0,
                 default_angle:  float = 90.0,
                 name:           str   = "SERVO"):

        self._log          = log.get_logger(name)
        self._angle_min    = float(angle_min)
        self._angle_max    = float(angle_max)
        self._default      = float(default_angle)
        self._angle_f      = float(default_angle)   # état interne flottant

        self._hw = adafruit_servo.Servo(
            pca_instance.channels[channel],
            min_pulse=SERVO_MIN_PULSE_US,
            max_pulse=SERVO_MAX_PULSE_US,
            actuation_range=SERVO_RANGE_DEG
        )

        self._write_hw(self._default)
        time.sleep(SERVO_INIT_DELAY_S)
        self._log.info("Initialisé canal %d → %.1f° (min=%.1f°, max=%.1f°)",
                       channel, self._default, angle_min, angle_max)

    @property
    def angle(self) -> float:
        return self._angle_f

    def _clamp(self, value: float) -> float:
        return max(self._angle_min, min(self._angle_max, value))

    def _write_hw(self, angle: float) -> None:
        clamped = self._clamp(angle)
        self._hw.angle = clamped
        self._angle_f  = clamped

    def set_angle(self, angle: float) -> None:
        self._write_hw(angle)

    def move_to(self,
                target:     float,
                step:       float = SERVO_STEP_DEG,
                step_delay: float = SERVO_STEP_DELAY_S) -> None:
        target = self._clamp(target)
        if abs(target - self._angle_f) < step:
            self._write_hw(target)
            return

        direction = 1.0 if target > self._angle_f else -1.0
        while abs(target - self._angle_f) > step:
            self._write_hw(self._angle_f + direction * step)
            time.sleep(step_delay)

        self._write_hw(target)  # snap final précis

    def center(self) -> None:
        self.move_to(self._default)
        time.sleep(0.3)

    def reset(self) -> None:
        self._write_hw(self._default)
        self._log.debug("Reset → %.1f°", self._default)



#  HARDWARE — TÊTE (PAN / TILT / ROUE)
class Head:
    """Tête du robot : 3 servos couplés montés sur PCA9685."""

    def __init__(self, pca_instance: PCA9685):
        self._log = log.get_logger("HEAD")

        self.wheel      = ServoMotor(pca_instance, CHANNEL_SERVO_WHEEL,
                                     WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX,
                                     WHEEL_ANGLE_CENTER, "WHEEL")
        self.horizontal = ServoMotor(pca_instance, CHANNEL_SERVO_HORIZONTAL,
                                     HEAD_ANGLE_MIN, HEAD_ANGLE_MAX,
                                     HEAD_ANGLE_CENTER, "PAN")
        self.vertical   = ServoMotor(pca_instance, CHANNEL_SERVO_VERTICAL,
                                     HEAD_ANGLE_MIN, HEAD_ANGLE_MAX,
                                     HEAD_ANGLE_CENTER, "TILT")

    def steer(self, angle: float) -> None:
        wheel_angle = max(WHEEL_ANGLE_MIN, min(WHEEL_ANGLE_MAX, angle))
        head_angle  = max(HEAD_ANGLE_MIN,  min(HEAD_ANGLE_MAX,  angle))
        self.wheel.set_angle(wheel_angle)
        self.horizontal.set_angle(head_angle)
        self._log.debug("steer → roue=%.1f° pan=%.1f°", wheel_angle, head_angle)

    def steer_left(self, intensity: float = STEER_SOFT_DEG) -> None:
        self.steer(WHEEL_ANGLE_CENTER + intensity)

    def steer_right(self, intensity: float = STEER_SOFT_DEG) -> None:
        self.steer(WHEEL_ANGLE_CENTER - intensity)

    def steer_center(self) -> None:
        self.steer(WHEEL_ANGLE_CENTER)

    def set_angle_motor(self, channel_sevro : int, angle: float) -> None:
        match channel_sevro:
            case 0:
                wheel_angle = max(WHEEL_ANGLE_MIN, min(WHEEL_ANGLE_MAX, angle))
                self._log.info("wheel_angle=%d, actual_angle=%d", wheel_angle, self.wheel.angle)
                self.wheel.set_angle(wheel_angle)
                self._log.info("wheel_angle=%d, actual_angle=%d", wheel_angle, self.wheel.angle)
            case 1:
                head_angle = max(HEAD_ANGLE_MIN, min(HEAD_ANGLE_MAX, angle))
                self.horizontal.set_angle(head_angle)
            case 2:
                head_angle = max(HEAD_ANGLE_MIN, min(HEAD_ANGLE_MAX, angle))
                self.vertical.set_angle(head_angle)
            case _:
                self._log.debug("servoID -> Not existing : servoID=%d", channel_sevro)

    def reset(self) -> None:
        self.wheel.reset()
        self.horizontal.reset()
        self.vertical.reset()
        self._log.info("Reset → tous les servos au centre")

    def test(self) -> None:
        self._log.info("Test — test les servos dans differentes positions défini")
        self.set_angle_motor(1, 50)
        self.set_angle_motor(2, 50)
        self.set_angle_motor(0, 35)
        time.sleep(0.5)
        self.reset()
        time.sleep(0.5)
        self.set_angle_motor(1, 140)
        self.set_angle_motor(2, 140)
        self.set_angle_motor(0, 125)
        time.sleep(0.5)
        self._log.info("Test — test fini !")

    def test_steer(self):
        self._log.info("Test — test les servos dans differentes positions défini")
        self.steer_left(40)
        time.sleep(0.5)
        self.steer_right(40)
        time.sleep(0.5)
        self._log.info("Test — test fini !")

    def get_max_range(self):
        self._log.info("Test — Pour avoir la max range des roues")
        self.steer_left(45)
        time.sleep(0.5)
        for i in range(90):
            self.steer_left(self.wheel.angle + 1)
            self._log.info("Wheel angle :",self.wheel.angle)
            time.sleep(0.5)

    def shutdown(self) -> None:
        self._log.info("Shutdown — recentrage des servos…")
        self.horizontal.center()
        self.vertical.center()
        self.wheel.center()
        self.reset()

if __name__ == "__main__":

    head = Head(SERVO_PCA)
    try:
        head.get_max_range()
        head.shutdown()
    except KeyboardInterrupt:
        head.shutdown()
        print("\nProgram terminated. Goodbye!")
        print("Program developed by Team C - MasterCamp SE 2026.")