from enum import IntEnum
from adafruit_pca9685 import PCA9685
from adafruit_motor import motor
import time
from typing import Optional
from board import SCL, SDA
import busio

import logging.logger as log

# ── Constantes ──────────────────────────────────────────────────────
MOTOR_RAMP_MAX_TIME_S = 1.0  # durée max de la rampe d'accélération
MOTOR_RAMP_STEPS      = 10   # nombre de pas par seconde de rampe

CHANNEL_MOTOR_IN1        = 15  # Moteur DC pôle +
CHANNEL_MOTOR_IN2        = 14  # Moteur DC pôle −

# SPEED FOR LINE FOLLOWING
SPEED_NORMAL_PCT      = 30   # % puissance — ligne droite
SPEED_TURNING_PCT     = 20   # % puissance — virage doux
SPEED_ADJUSTING_PCT   = 40
SPEED_BACKWARD        = 30   # % puissance — utilisé généralement pour aller en arrière
SPEED_HIGH = 40

class Direction(IntEnum):
    """Sens de marche du moteur DC."""
    FORWARD  =  1
    BACKWARD = -1

#  HARDWARE — MOTEUR DC
class DCMotor:
    """Moteur de traction arrière via PCA9685 + adafruit_motor."""

    def __init__(self, pca_instance: PCA9685):
        self._log       = log.get_logger("MOTOR")
        self._motor     = motor.DCMotor(
            pca_instance.channels[CHANNEL_MOTOR_IN1],
            pca_instance.channels[CHANNEL_MOTOR_IN2]
        )
        self._motor.decay_mode = motor.SLOW_DECAY

        self._speed     = 0.0
        self._direction = Direction.FORWARD
        self._stopped   = True

        self._stop_immediate()
        self._log.info("Moteur DC initialisé (IN1=%d, IN2=%d)",
                       CHANNEL_MOTOR_IN1, CHANNEL_MOTOR_IN2)

    @staticmethod
    def _pct_to_throttle(speed_pct: float) -> float:
        return max(0.0, min(1.0, speed_pct / 100.0))

    def _apply_power(self, direction: Direction, speed_pct: float) -> None:
        throttle = self._pct_to_throttle(speed_pct)
        if direction == Direction.BACKWARD:
            throttle = -throttle
        self._motor.throttle = throttle

    def _ramp(self, target_speed: float, ramp_time: float = MOTOR_RAMP_MAX_TIME_S) -> None:
        ramp_time = min(ramp_time, MOTOR_RAMP_MAX_TIME_S)
        steps     = max(1, round(ramp_time * MOTOR_RAMP_STEPS))
        delta     = (target_speed - self._speed) / steps

        for _ in range(steps):
            self._speed += delta
            self._apply_power(self._direction, self._speed)
            time.sleep(1.0 / MOTOR_RAMP_STEPS)

        self._speed = target_speed
        self._apply_power(self._direction, self._speed)

    def _stop_immediate(self) -> None:
        self._speed            = 0.0
        self._motor.throttle   = 0
        self._stopped          = True

    def stop(self) -> None:
        self._ramp(0.0, ramp_time=0.2)
        self._stop_immediate()
        self._log.debug("Stop")

    def reset(self) -> None:
        self._stop_immediate()
        self._log.debug("Reset")

    def drive(self,
              direction:   Direction = Direction.FORWARD,
              speed_pct:   float     = SPEED_NORMAL_PCT,
              duration_s:  Optional[float] = None,
              slow:        bool      = False,
              fast_accel: bool = False) -> None:
        if direction != self._direction:
            self.stop()
        self._direction = direction
        self._stopped   = False

        if slow:
            self._apply_power(self._direction, 25.0)
        elif duration_s is not None and duration_s < 1.0:
            self._ramp(speed_pct, ramp_time=duration_s)
            self.stop()
        elif fast_accel:
            self._ramp(speed_pct, ramp_time=0.1)
        else:
            self._ramp(speed_pct)

if __name__ == '__main__':
    # test automatique, avancé reculé et arrêt, le tout en douceur
    # Pour le contrôle avec le clavier, voir le fichier keyboard_control.py
    i2c = busio.I2C(SCL, SDA)
    PCA_ADDRESS = 0x5F
    pca = PCA9685(i2c, address=PCA_ADDRESS)
    motor = DCMotor(pca)
    try:
        for i in range(10):
            motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            print("Forward")
            time.sleep(3)
            motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT)
            print("Backward")
            time.sleep(3)

            motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
            print("Forward then stop")
            time.sleep(2)
            motor.stop()
            time.sleep(2)

            motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT)
            print("Backward then stop")
            time.sleep(2)
            motor.stop()
            time.sleep(2)
        motor.stop()
    except KeyboardInterrupt:
        motor.stop()
