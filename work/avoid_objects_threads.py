import time

from t11_robot import Robot


from t3_servomotors import WHEEL_ANGLE_CENTER, HEAD_ANGLE_CENTER, STEER_SOFT_DEG, STEER_HARD_DEG, CHANNEL_SERVO_VERTICAL, CHANNEL_SERVO_HORIZONTAL, CHANNEL_SERVO_WHEEL
from t4_dc_motor import Direction, SPEED_NORMAL_PCT
from t6_line_tracking import LinePosition

# Constantes

# ── Buzzer ─────────────────────────────────────────────────────────
# Son joué pendant les manœuvres de récupération (recul + virage quand
# la ligne est perdue) :
#   None      -> silence
#   "MII"     -> thème MII (comme en roulage normal)
#   "POLICE"  -> sirène POLICE (comme en urgence obstacle)
LINE_LOST_SOUND = "MII"

CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs

# ═══════════════════════════════════════════════════════════════════
#  THREADS
# ═══════════════════════════════════════════════════════════════════


MODE_AVOID_LINE = True
MODE_AVOID_OBJ = False
MODE = MODE_AVOID_OBJ

# CONSTANTS AND VARIABLES FOR AVOID OBJECTS
scan = []

SCAN_ANGLE = 65
SCAN_DIST_ACTION = 18 # in cm !!!

TURN_RIGHT = True
TURN_LEFT = False
turning_angle = 30
BYPASS_RIGHT_ANGLE = WHEEL_ANGLE_CENTER - turning_angle
BYPASS_LEFT_ANGLE = WHEEL_ANGLE_CENTER + turning_angle


AVOID_OBJ_SPEED = SPEED_NORMAL_PCT * 0.35
BYPASS_SPEED = SPEED_NORMAL_PCT * 0.85

SCAN_STEP = 10
SCAN_WAIT_TIME = 0.2

# CONSTANTS AND VARIABLES FOR AVOID LINES
AVOID_LINE_TURN_SPEED = AVOID_OBJ_SPEED*2


def thread_ultrasonic_scanning(robot: Robot, interval: float) -> None:
    """Lit le capteur ultrason en boucle en balayant de droite à gauche et met à jour la variable global scan."""
    global scan

    def scan_cm() -> list:
        # scanning from left to right using the ultrasonic module
        data = []
        start_position = int(HEAD_ANGLE_CENTER - (SCAN_ANGLE/2))  # right
        end_position = int(HEAD_ANGLE_CENTER + (SCAN_ANGLE/2))    # left
        robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, HEAD_ANGLE_CENTER + 5) # looking forward vertically
        robot.head.set_angle_motor(CHANNEL_SERVO_HORIZONTAL, start_position)      #setting at start position
        time.sleep(0.2) # waiting head to be ready
        data_str = ""
        for angle in range(start_position, end_position+1, SCAN_STEP): # scanning from left ro right
            robot.head.set_angle_motor(CHANNEL_SERVO_HORIZONTAL, angle)
            time.sleep(SCAN_WAIT_TIME)
            distance_cm = robot.ultrasonic.read_mm()/10
            data.append(distance_cm)
            data_str = str(round(distance_cm, 1)) + " " + data_str
        # print(data_str)
        robot.head.set_angle_motor(CHANNEL_SERVO_HORIZONTAL, HEAD_ANGLE_CENTER)
        return data

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        scan = scan_cm() # scanning and putting the result in the global scan variable
        if MODE == MODE_AVOID_LINE:
            robot.head.set_angle_motor(CHANNEL_SERVO_VERTICAL, 60)  # looking downward to see the blue square
            robot.head.set_angle_motor(CHANNEL_SERVO_HORIZONTAL, HEAD_ANGLE_CENTER+5)
            return

def thread_line_detect_avoid(robot: Robot, interval: float) -> None:
    """
    Lit l'action décodée des capteurs en boucle (via read_action)
    et met à jour directement l'action sur le RobotState.
    """
    global MODE

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        # Capture matérielle et décodage atomique (Hors du Lock pour optimiser)
        current_action = robot.line_tracker.read_action()

        with robot.state.lock:
            robot.state.line_action = current_action
        if current_action != LinePosition.LINE_LOST:
            MODE = MODE_AVOID_LINE
        time.sleep(interval)


def thread_avoid_line_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision pour le suivi de cercle :
    lit l'action synthétisée, décide et pilote les moteurs.
    """
    global MODE

    def action_direction(action: LinePosition) -> str:
        directions = {
            LinePosition.STRAIGHT: "tout droit",
            LinePosition.TURN_RIGHT_HARD: "à gauche (léger)",
            LinePosition.TURN_RIGHT_SOFT: "à gauche (fort)",
            LinePosition.TURN_LEFT_HARD: "à droite (léger)",
            LinePosition.TURN_LEFT_SOFT: "à droite (fort)",
            LinePosition.INTERSECTION: "ambigu",
            LinePosition.LINE_LOST: "recherche",
        }
        return directions.get(action, "inconnue")

    # boucle pour attendre la première détection de la ligne
    while True:
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
        if robot.state.line_action != LinePosition.LINE_LOST:
            break
        time.sleep(0.05)

    last_action = None
    while True:
        # ── Lecture atomique de l'état simplifié ──────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop

        # ── Arrêt d'urgence obstacle (Priorité 1) ─────────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            time.sleep(interval)
            continue

        # Lire les capteurs bruts (gauche, milieu, droit)
        current_action = robot.state.line_action
        if current_action != last_action:
            print("ACTION LINE:", action_direction(current_action))

        # Comportement d'ÉVITEMENT (s'inspire de t7 mais inversé)
        # Priorité : détection droite -> tourner à gauche; détection gauche -> tourner à droite
        if current_action == LinePosition.TURN_LEFT_HARD:
            # Approche depuis la droite -> tourner doucement à gauche
            last_action = current_action
            robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER - STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)

        elif current_action == LinePosition.TURN_LEFT_SOFT:
            # Trop à droite -> tourner fort à gauche
            last_action = current_action
            robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER - STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)

        elif current_action == LinePosition.TURN_RIGHT_HARD:
            # Approche depuis la gauche -> tourner doucement à droite
            last_action = current_action
            robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER + STEER_SOFT_DEG)
            robot.motor.drive(Direction.FORWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)

        elif current_action == LinePosition.TURN_RIGHT_SOFT:
            # Trop à gauche -> tourner fort à droite
            last_action = current_action
            robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER + STEER_HARD_DEG)
            robot.motor.drive(Direction.FORWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)

        elif current_action ==  LinePosition.STRAIGHT or current_action == LinePosition.INTERSECTION:
            # Ligne centrée -> tout droit
            if last_action == LinePosition.TURN_RIGHT_HARD or last_action == LinePosition.TURN_RIGHT_SOFT:
                # backward maneuver
                print("back maneuver")
                robot.motor.stop()
                robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER - STEER_HARD_DEG)
                time.sleep(0.2)
                robot.motor.drive(Direction.BACKWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)
                time.sleep(1.5)
            elif last_action == LinePosition.TURN_LEFT_HARD or last_action == LinePosition.TURN_LEFT_SOFT:
                print("back maneuver")
                robot.motor.stop()
                robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER + STEER_HARD_DEG)
                time.sleep(0.2)
                robot.motor.drive(Direction.BACKWARD, AVOID_LINE_TURN_SPEED, fast_accel=True)
                time.sleep(1.5)
            # in all cases
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED, fast_accel=True)

        else:
            # Aucun capteur -> avancer doucement ou chercher
            robot.head.steer_center()
            robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED, fast_accel=True)

        time.sleep(interval)

    # ── Arrêt propre en fin de thread ─────────────────────────────
    robot.motor.stop()
    robot.head.steer_center()


def thread_object_controller(robot: Robot, interval: float) -> None:
    """
    Boucle de décision : lit l'action synthétisée, décide et pilote les moteurs.
    """

    def bypass_side(index):
        """Determine if we should bypass by the left of the right, given an index"""
        angle = HEAD_ANGLE_CENTER - (SCAN_ANGLE / 2) + index * SCAN_STEP
        if angle <= HEAD_ANGLE_CENTER:  # if object on the right
            return TURN_LEFT
        else:  # object on the left
            return TURN_RIGHT

    def bypass(robot, bypass_direction, obj_idx, distance_cm):
        """Bypassing an object by the left or by the right"""

        def get_absolute_angle(idx, bypass_side):
            """From a given distance in a scan we determine the absolute angle from the front of the robot"""
            angle = HEAD_ANGLE_CENTER - SCAN_ANGLE / 2 + idx * SCAN_STEP
            if bypass_side == TURN_RIGHT:  # meaning object on left
                return angle - HEAD_ANGLE_CENTER
            else:  # meaning object on right
                return HEAD_ANGLE_CENTER - angle

        if bypass_direction == TURN_RIGHT:  # good direction from indications
            turn = BYPASS_RIGHT_ANGLE
            counter_turn = BYPASS_LEFT_ANGLE
        else:
            turn = BYPASS_LEFT_ANGLE
            counter_turn = BYPASS_RIGHT_ANGLE

        obj_angle = get_absolute_angle(obj_idx, bypass_direction)
        # print("obj angle ", obj_angle, " obj_idx ", obj_idx, " bypass dir ", bypass_direction)
        ratio_angle = obj_angle / (SCAN_ANGLE / 2)
        ratio_distance = distance_cm / SCAN_DIST_ACTION
        # print("ratio_angle ", str(obj_angle),"/", str(SCAN_ANGLE/2),"=", ratio_angle, " ratio_distance = ",str(distance_cm),"/",str(SCAN_DIST_ACTION), ratio_distance)

        # backward a bit first
        robot.motor.drive(Direction.BACKWARD, SPEED_NORMAL_PCT * 0.5)
        robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER)
        backward_sleep_time = max(0, ((1 - ratio_angle) + (2 - ratio_distance * 2))/2)  # between 0 and 1 seconds, inversly proportional to the distance and to the angle
        time.sleep(backward_sleep_time)
        print("backward_sleep_time ", backward_sleep_time)
        # time.sleep(0.1 * (1 / (distance_cm/10) ) ) # adjust how much we go backward depending on the distance to the obstacle
        robot.motor.stop()

        # the sleep time allow to do a bigger or smaller maneuver depending on where is the obj (obj_angle)
        if obj_angle <= 22:
            print("object close")
            sleep_time = 2.5
        elif obj_angle <= 27:
            print("object mid")
            sleep_time = 2
        else:
            print("object far")
            sleep_time = 1
        # sleep_time = 0.1 + 0.1 * (SCAN_ANGLE/2 - obj_angle)
        # sleep_time = 2 * (SCAN_ANGLE/2 - obj_angle)
        # sleep_time = 2

        if MODE == MODE_AVOID_LINE: return

        # turn
        robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, turn)
        time.sleep(0.3)
        robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
        time.sleep(sleep_time)

        robot.motor.stop()

        if MODE == MODE_AVOID_LINE: return

        # counter_turn
        robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, counter_turn)
        time.sleep(0.3)
        robot.motor.drive(Direction.FORWARD, BYPASS_SPEED)
        time.sleep(1 * sleep_time)
        robot.motor.stop()

        if MODE == MODE_AVOID_LINE: return

        # reset T pose
        robot.motor.stop()
        robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER)

    def right_bypass(idx, dist):
        print("turn right")
        robot.motor.stop()
        # input("next action")
        bypass(robot, TURN_RIGHT, idx, dist)

    def left_bypass(idx, dist):
        print("turn left")
        robot.motor.stop()
        # input("next action")
        bypass(robot, TURN_LEFT, idx, dist)

    # CONTROLLER MAIN LOGIC
    global scan
    try:
        driving = False
        while True:
            with robot.state.lock: # stopping the loop when program is stopped
                if not robot.state.running:
                    break
            if MODE == MODE_AVOID_LINE: return

            # DRIVING AVOID OBJECTS LOGIC
            if scan:
                actual_scan = scan.copy()
                min_dist = min(actual_scan)
                if min_dist <= SCAN_DIST_ACTION:
                    # doing a second scan when we are stopped
                    robot.motor.stop()
                    time.sleep(SCAN_ANGLE/SCAN_STEP * SCAN_WAIT_TIME +0.3)
                    actual_scan = scan
                    min_dist = min(actual_scan)
                    driving = False

                    min_dist_idx = scan.index(min_dist)

                    if MODE == MODE_AVOID_LINE: return

                    print(" None")
                    print(bypass_side(min_dist_idx))
                    if bypass_side(min_dist_idx) == TURN_RIGHT:
                        right_bypass(min_dist_idx, min_dist)
                    else:
                        left_bypass(min_dist_idx, min_dist)

                elif not driving:
                    if MODE == MODE_AVOID_LINE: return
                    print("drive")
                    robot.motor.stop()
                    driving = True
                    robot.head.set_angle_motor(CHANNEL_SERVO_WHEEL, WHEEL_ANGLE_CENTER)
                    robot.motor.drive(Direction.FORWARD, AVOID_OBJ_SPEED)
            else:
                print("no data yet")

            time.sleep(interval)
    except KeyboardInterrupt:
        # ── Arrêt propre en fin de thread ─────────────────────────────
        robot.motor.stop()
        robot.head.steer_center()
