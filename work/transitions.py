import time

from t11_robot import Robot
import camera_line3


def calibration_sequence_IR_to_obstacles(robot_instance: Robot) -> None:
    """Séquence de mouvements pour se préparer à l'évitement d'obstacles."""
    global log, target_step
    log.info("[SYS] INIT_CALIBRATION: OBSTACLES -> Réinitialisation odométrie & capteurs")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 90)
    time.sleep(0.3)

    log.info("[SYS] CALIBRATION_OK: OBSTACLES -> Transition demandée vers 'Obstacles'")
    target_step = "Obstacles"


def calibration_sequence_obstacles_to_camera_line(robot_instance: Robot) -> None:
    """Séquence pour baisser la tête de la caméra vers le sol pour le suivi de ligne rouge."""
    global log, target_step
    log.info("[SYS] INIT_CALIBRATION: LIGNE_ROUGE -> Inclinaison caméra basse (60deg)")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(2, 60)
    time.sleep(0.3)

    log.info("[SYS] CALIBRATION_OK: LIGNE_ROUGE -> Transition demandée vers 'Camera Line'")
    target_step = "Camera Line"
    with robot_instance.state.lock:
        robot_instance.state.running = True


def calibration_sequence_camera_to_labyrinth(robot_instance: Robot) -> None:
    """Séquence de calibration matérielle pour le labyrinthe."""
    global log, target_step
    log.info("[SYS] INIT_CALIBRATION: LABYRINTHE -> Alignement roues & caméra")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 110)
    robot_instance.motor.drive(camera_line3.Direction.FORWARD, 20, fast_accel=True)
    time.sleep(0.5)
    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 90)

    log.info("[SYS] CALIBRATION_OK: LABYRINTHE -> Transition demandée vers 'Labyrinthe'")
    target_step = "Labyrinthe"

