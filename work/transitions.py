import time
from t11_robot import Robot
import camera_line3


def calibration_sequence_IR_to_obstacles(robot_instance: Robot) -> str:
    """Séquence de mouvements pour se préparer à l'évitement d'obstacles."""
    print("[SYS] INIT_CALIBRATION: OBSTACLES -> Réinitialisation odométrie & capteurs")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 90)
    robot_instance.motor.drive(camera_line3.Direction.FORWARD, 20, fast_accel=True)
    time.sleep(0.4)
    robot_instance.motor.reset()


    print("[SYS] CALIBRATION_OK: OBSTACLES -> Passage automatique à 'Obstacles'")
    return "Obstacles"


def calibration_sequence_obstacles_to_camera_line(robot_instance: Robot) -> str:
    """Séquence pour baisser la tête de la caméra vers le sol pour le suivi de ligne rouge."""
    print("[SYS] INIT_CALIBRATION: LIGNE_ROUGE -> Inclinaison caméra basse (60deg)")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(2, 60)
    time.sleep(1.3)

    print("[SYS] CALIBRATION_OK: LIGNE_ROUGE -> Passage automatique à 'Camera Line'")
    return "Camera Line"


def calibration_sequence_camera_to_labyrinth(robot_instance: Robot) -> str:
    """Séquence de calibration matérielle pour le labyrinthe."""
    print("[SYS] INIT_CALIBRATION: LABYRINTHE -> Alignement roues & caméra")

    with robot_instance.state.lock:
        robot_instance.state.running = False

    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 110)
    robot_instance.motor.drive(camera_line3.Direction.FORWARD, 20, fast_accel=True)
    time.sleep(0.5)
    robot_instance.motor.reset()
    robot_instance.head.set_angle_motor(0, 90)

    print("[SYS] CALIBRATION_OK: LABYRINTHE -> Passage automatique à 'Labyrinthe'")
    return "Labyrinthe"