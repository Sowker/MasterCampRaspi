import time
from picamera2 import Picamera2

from line_following.t11_robot import Robot

from components.t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from components.t4_dc_motor import Direction

SPEED_NORMAL_PCT      = 30   # % puissance — ligne droite
SPEED_TURNING_PCT     = 20   # % puissance — virage doux
SPEED_ADJUSTING_PCT   = 40
SPEED_BACKWARD        = 30   # % puissance — utilisé généralement pour aller en arrière
SPEED_HIGH = 40


from labyrinthe.CameraDetection import get_direction, adjust_position

def get_arrow_detection(camera : Picamera2)->Direction:
    return get_direction(camera)

def L_turn(robot : Robot, direction : str) -> None:
    robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
    time.sleep(0.6)
    if direction == "left":
        for i in range(5):
            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.20)

            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.15)

        robot.head.steer_left(STEER_HARD_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
        time.sleep(0.18)

        robot.head.steer_center()
        # robot.head.set_angle_motor(0, 90 - 10)
        robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
        time.sleep(0.15)


    elif direction == "right":
        for i in range(5):
            robot.head.steer_right(STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.20)

            robot.head.steer_left(STEER_HARD_DEG)
            robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
            time.sleep(0.15)

        robot.head.steer_right(STEER_HARD_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
        time.sleep(0.18)

        robot.head.steer_center()
        # robot.head.set_angle_motor(0, 90 - 10)
        robot.motor.drive(Direction.BACKWARD, SPEED_HIGH, fast_accel=True)
        time.sleep(0.15)
    else:
        pass

def thread_drive(robot: Robot, interval: float, camera : Picamera2) -> None:
    log = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)
    # robot.head.set_angle_motor(0, 85)
    robot.head.steer_center()
    while True:
        # ── 1. Lecture de l'état actuel ───────────────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop

        # ── 2. Gestion de l'urgence (Priorité Absolue) ────────────
        if emergency:
            correction = adjust_position(camera)
            if correction != "straight":
                if correction == "left":
                    robot.head.set_angle_motor(0, 90 - STEER_SOFT_DEG)
                elif correction == "right":
                    robot.head.set_angle_motor(0, 90 + STEER_SOFT_DEG)
                robot.motor.drive(Direction.BACKWARD, SPEED_TURNING_PCT, fast_accel=True)
                time.sleep(0.5)
                robot.head.steer_center()
                #robot.head.set_angle_motor(0, 90 - 10)
                robot.motor.drive(Direction.BACKWARD,SPEED_TURNING_PCT, fast_accel=True)
                time.sleep(0.6)
            else :
                robot.motor.stop()
                robot.head.steer_center()
                # robot.head.set_angle_motor(0, 90 - 10)
                log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
                direction = get_arrow_detection(camera)
                L_turn(robot, direction)
        else:
            #  ── 3. Adjust the position of the robot to be straigth ────────────
            corretion = adjust_position(camera)
            if corretion == "right":
                robot.head.set_angle_motor(0, 90 - STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
            elif corretion == "left":
                robot.head.set_angle_motor(0, 90 + STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
            else:
                robot.head.set_angle_motor(1, 100)
                robot.head.steer_center()
                # robot.head.set_angle_motor(0, 85)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

def thread_ultrasonic(robot: Robot, interval: float) -> None:
    """
    Lit le capteur ultrason en boucle pour détecter les obstacles.
    Met à jour l'état d'urgence du robot si un objet est trop près.
    """
    log = logger.get_logger("US")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        # Vérifie si le robot est toujours censé fonctionner
        with robot.state.lock:
            if not robot.state.running:
                break

        # Lecture de la distance de l'obstacle en millimètres
        dist_mm = robot.ultrasonic.read_mm()

        # Mise à jour sécurisée de l'état global du robot
        with robot.state.lock:
            robot.state.distance_mm = dist_mm
            # Déclenche l'arrêt d'urgence si la distance est sous le seuil critique
            robot.state.emergency_stop = dist_mm < 350

        time.sleep(interval)

    log.info("Thread arrêté")
