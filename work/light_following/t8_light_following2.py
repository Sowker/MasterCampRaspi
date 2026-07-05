import time
import smbus
from collections import namedtuple

# Resultat de lecture : valeurs des deux capteurs + direction deduite
LightReading = namedtuple("LightReading", ["left", "right", "direction"])

# 5 angles de braquage possibles (repris de t3_servomotors.py)
WHEEL_ANGLE_CENTER = 90
STEER_SOFT_DEG     = 15
STEER_HARD_DEG     = 35

STEER_HARD_LEFT  = WHEEL_ANGLE_CENTER - STEER_HARD_DEG   # 55
STEER_SOFT_LEFT  = WHEEL_ANGLE_CENTER - STEER_SOFT_DEG   # 75
STEER_CENTER     = WHEEL_ANGLE_CENTER                    # 90
STEER_SOFT_RIGHT = WHEEL_ANGLE_CENTER + STEER_SOFT_DEG   # 105
STEER_HARD_RIGHT = WHEEL_ANGLE_CENTER + STEER_HARD_DEG   # 125


class ADS7830:
    # ADC I2C, 8 canaux (0-7), valeurs 0-255

    def __init__(self, address: int = 0x48):
        self.cmd     = 0x84
        self.bus     = smbus.SMBus(1)
        self.address = address

    def analogRead(self, chn: int) -> int:
        return self.bus.read_byte_data(
            self.address,
            self.cmd | (((chn << 2 | chn >> 1) & 0x07) << 4),
        )


class LightFollowingModule:
    def __init__(self, ch_left: int = 0, ch_right: int = 1, threshold: int = 15):
        self.adc       = ADS7830()
        self.ch_left   = ch_left
        self.ch_right  = ch_right
        self.threshold = threshold

    def read(self) -> LightReading:
        # lit les deux capteurs et determine de quel cote vient la lumiere
        left  = self.adc.analogRead(self.ch_left)
        right = self.adc.analogRead(self.ch_right)

        # inverse : valeur ADC plus faible = plus de lumiere de ce cote
        diff = right - left
        if diff > self.threshold:
            direction = "LEFT"
        elif diff < -self.threshold:
            direction = "RIGHT"
        else:
            direction = "CENTER"

        return LightReading(left=left, right=right, direction=direction)

    def get_steer_angle(self, reading: LightReading = None,
                         soft_threshold: int = 15,
                         hard_threshold: int = 50) -> float:
        # convertit l'ecart de lumiere en un des 5 angles de braquage
        if reading is None:
            reading = self.read()

        # inverse : valeur ADC plus faible = plus de lumiere de ce cote
        diff = reading.right - reading.left

        if diff > hard_threshold:
            return STEER_HARD_LEFT
        elif diff > soft_threshold:
            return STEER_SOFT_LEFT
        elif diff < -hard_threshold:
            return STEER_HARD_RIGHT
        elif diff < -soft_threshold:
            return STEER_SOFT_RIGHT
        else:
            return STEER_CENTER


if __name__ == "__main__":
    try:
        from components.t3_servomotors import (
            ServoMotor, SERVO_PCA, CHANNEL_SERVO_WHEEL,
            WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX, WHEEL_ANGLE_CENTER,
        )
        from components.t4_dc_motor import DCMotor, Direction, SPEED_NORMAL_PCT
        _HAS_HARDWARE = True
    except ImportError:
        _HAS_HARDWARE = False

    sensor = LightFollowingModule(ch_left=0, ch_right=1)

    print("=== Suivi de lumiere -> braquage + avance (Ctrl+C pour arreter) ===")
    print(f"{'Gauche':>7}  {'Droite':>7}  {'Ecart':>6}  Angle roue")
    print("-" * 38)

    if _HAS_HARDWARE:
        # un seul servo cree (canal roue) -> ne touche pas a la tete
        wheel = ServoMotor(SERVO_PCA, CHANNEL_SERVO_WHEEL,
                           WHEEL_ANGLE_MIN, WHEEL_ANGLE_MAX,
                           WHEEL_ANGLE_CENTER, "WHEEL")
        motor = DCMotor(SERVO_PCA)
        # une seule rampe au demarrage (sinon drive() relance une rampe a
        # chaque iteration de la boucle -> moteur qui "bippe")
        motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT)
    else:
        wheel = motor = None
        print("(t3_servomotors / t4_dc_motor introuvables — affichage seul)\n")

    try:
        while True:
            try:
                r     = sensor.read()
                angle = sensor.get_steer_angle(r)
                print(f"{r.left:>7}  {r.right:>7}  {r.left - r.right:>6}  {angle:>5.0f}°", end="\r")

                if wheel is not None:
                    wheel.set_angle(angle)

            except OSError as e:
                print(f"\n[I2C] ADS7830 inaccessible (0x48) : {e}")
                time.sleep(0.5)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("\n\nArret — recentrage et coupure moteur...")
        if wheel is not None:
            wheel.center()
            motor.stop()
        print("Programme developpe par l'Equipe C - MasterCamp SE 2026.")
