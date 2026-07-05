import time
from typing import Optional

from line_following.t11_robot import Robot
import logging.logger

from components.t3_servomotors import STEER_HARD_DEG, STEER_SOFT_DEG
from components.t4_dc_motor import Direction
from components.t6_line_tracking import LinePosition
from line_following.t11_buzzer_Sirene import POLICE, MII, play

# SPEED FOR LINE FOLLOWING
SPEED_NORMAL_PCT      = 30   # % puissance — ligne droite
SPEED_TURNING_PCT     = 20   # % puissance — virage doux
SPEED_ADJUSTING_PCT   = 40
SPEED_BACKWARD        = 30   # % puissance — utilisé généralement pour aller en arrière
SPEED_HIGH = 40

# ===================================================================
# Constantes de configuration
# ===================================================================

# ── Buzzer ─────────────────────────────────────────────────────────
# Son joué pendant les manœuvres de récupération (recul + virage quand la ligne est perdue) :
#   None      -> silence
#   "MII"     -> thème MII (comme en roulage normal)
#   "POLICE"  -> sirène POLICE (comme en urgence obstacle)
LINE_LOST_SOUND = "MII"

# ── Temporisations ─────────────────────────────────────────────────
TIME_LOST = 1.75  # Temps d'attente/délai avant de réagir à la perte de ligne
TIME_POST_MANUVER = 0.1  # Temps alloué pour se stabiliser après avoir retrouvé la ligne

CTRL_INTERVAL_S = 0.05  # s — période du thread contrôleur (cerveau)
SENSOR_INTERVAL_S = 0.05  # s — période des threads capteurs (yeux/oreilles)


# ===================================================================
#  THREADS (Tâches exécutées en parallèle)
# ===================================================================

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
            robot.state.emergency_stop = dist_mm < robot._obstacle_threshold_mm

        time.sleep(interval)

    log.info("Thread arrêté")


def thread_line(robot: Robot, interval: float) -> None:
    """
    Surveille en permanence les capteurs de ligne sous le robot.
    Traduit les lectures physiques en actions (ex: tourner à gauche, tout droit).
    """
    log = logger.get_logger("LINE")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break

        # Capture matérielle hors du "lock" pour ne pas bloquer les autres threads
        current_action = robot.line_tracker.read_action()

        # Enregistre la position de la ligne dans l'état du robot
        with robot.state.lock:
            robot.state.line_action = current_action

        time.sleep(interval)

    log.info("Thread arrêté")


def thread_LED(robot: Robot, interval: float):
    """
    Gère la signalisation visuelle du robot selon son état :
    - Arrêt d'urgence ou ligne perdue -> Feux de détresse (Warning)
    - Virages -> Clignotant correspondant
    - Ligne droite -> LEDs éteintes
    """
    log = logger.get_logger("LED")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_front_state = None  # Mémorise le dernier état pour éviter les appels redondants

    while True:
        with robot.state.lock:
            if not robot.state.running:
                break
            action = robot.state.line_action
            emergency = robot.state.emergency_stop

        # Définition de l'état visuel en fonction des priorités
        if emergency:
            target_state = 'warning'
            robot.led.warning()
        elif action in (LinePosition.TURN_LEFT_SOFT, LinePosition.TURN_LEFT_HARD):
            target_state = 'left'
            robot.led.clignotant_gauche()
        elif action in (LinePosition.TURN_RIGHT_SOFT, LinePosition.TURN_RIGHT_HARD):
            target_state = 'right'
            robot.led.clignotant_droit()
        elif action == LinePosition.LINE_LOST:
            target_state = 'warning'
            robot.led.warning()
        else:  # Ligne droite ou Intersection
            target_state = None
            robot.led.arreter_clignotants()
            robot.led.arreter_warning()

        # Mise à jour des LEDs avant uniquement si l'état a changé
        if target_state != last_front_state:
            robot.front_leds.set_blink(target_state)
            last_front_state = target_state

        time.sleep(interval)

    robot.front_leds.cancel_blink()
    log.info("Thread arrêté")

def movement_post_manuver(robot: Robot, interval: float, log) -> None:
    if time.time() <= robot.state.post_time + TIME_POST_MANUVER:
        if robot.state.line_action == LinePosition.LINE_LOST:  # Si on reperd la ligne pendant la stabilisation
            log.info("Ligne reperdue, le dernier virage était " + str(robot.state.last_turn))
            robot.motor.stop()
            robot.head.steer_center()

            with robot.state.lock:
                robot.state.lost_time = time.time()
                robot.state.post_manuver = False
                robot.state.maneuver = True
        else:
            # La logique de conduite ici est identique à la conduite de base (voir plus bas)
            if robot.state.last_turn == 0:
                robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)

            elif robot.state.last_turn == -1:
                robot.head.steer_left(STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

            elif robot.state.last_turn == 1:
                robot.head.steer_right(STEER_SOFT_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

            elif robot.state.last_turn == -2:
                robot.head.steer_left(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

            elif robot.state.last_turn == 2:
                robot.head.steer_right(STEER_HARD_DEG)
                robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)

    else:
        with robot.state.lock:
            robot.state.post_manuver = False
            robot.state.maneuver = False

def basic_movement(robot: Robot, interval: float, log) -> None:
    # Logic based on the linerPosition -> action that the robot need to perform to follow it
    if robot.state.line_action in [LinePosition.STRAIGHT, LinePosition.INTERSECTION]:
        robot.head.steer_center()

        if robot.state.last_turn == 0:
            robot.motor.drive(Direction.FORWARD, SPEED_HIGH, fast_accel=True)
        else:
            robot.motor.drive(Direction.FORWARD, SPEED_NORMAL_PCT, fast_accel=True)

    elif robot.state.line_action == LinePosition.TURN_LEFT_SOFT:
        robot.head.steer_left(STEER_SOFT_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        robot.state.last_turn = -1

    elif robot.state.line_action == LinePosition.TURN_RIGHT_SOFT:
        robot.head.steer_right(STEER_SOFT_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        robot.state.last_turn = 1

    elif robot.state.line_action == LinePosition.TURN_LEFT_HARD:
        robot.head.steer_left(STEER_HARD_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        robot.state.last_turn = -2

    elif robot.state.line_action == LinePosition.TURN_RIGHT_HARD:
        robot.head.steer_right(STEER_HARD_DEG)
        robot.motor.drive(Direction.FORWARD, SPEED_TURNING_PCT, fast_accel=True)
        robot.state.last_turn = 2

    # --- PERTE DE LIGNE (Déclenchement manœuvre) ---
    else:  # LinePosition.LINE_LOST

        # detect if already in the lost state to not reset the timer
        if not (robot.state.already_lost):
            log.info("Ligne perdue, le dernier virage était " + str(robot.state.last_turn))
            robot.motor.stop()
            robot.head.steer_center()
            with robot.state.lock:
                robot.state.lost_time = time.time()
                robot.state.post_manuver = False
                robot.state.already_lost = True

        else:
            # We already have detected that we lost the line
            # Si la ligne vient d'être perdue, on attend un peu (TIME_LOST) avant de paniquer
            if time.time() <= robot.state.lost_time + TIME_LOST:
                '''
                robot.head.steer_center()
                robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                '''

                # Si on allait tout droit, on continue tout droit en espérant la retrouver (traits discontinus)
                if robot.state.last_turn in [0, 1, -1]:
                    robot.head.steer_center()
                    robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)

                # Si on tournait à gauche, on continue pour detecter une ligne
                elif robot.state.last_turn in [2, -2]:
                    with robot.state.lock:
                        robot.state.maneuver = True
                    '''
                    robot.head.steer_left(STEER_HARD_DEG)
                    robot.motor.drive(Direction.FORWARD, SPEED_BACKWARD, fast_accel=True)
                    '''

            else:
                # the timer runs out, we go to the manuver state do decide what to do
                with robot.state.lock:
                    robot.state.maneuver = True
                    robot.state.last_turn = 0

def movement_manuver(robot: Robot, interval: float, log) -> None:
    # Manuver
    if robot.state.line_action == LinePosition.LINE_LOST:
        # On recule en arc de cercle jusqu'à ce qu'un capteur touche à nouveau la ligne
        if robot.state.last_turn in [-1,-2]:  # On s'était perdu en tournant à gauche
            # On arrête le recul dès que le capteur droit voit la ligne fortement
            robot.head.steer_right(STEER_SOFT_DEG)  # On se redresse doucement
            robot.motor.drive(Direction.BACKWARD, SPEED_BACKWARD, fast_accel=True)

        elif robot.state.last_turn in [1,2]:  # On s'était perdu en tournant à droite
            # On arrête le recul dès que le capteur gauche voit la ligne fortement
            robot.head.steer_left(STEER_SOFT_DEG)
            robot.motor.drive(Direction.BACKWARD, SPEED_TURNING_PCT, fast_accel=True)

        elif robot.state.last_turn == 0:  # On s'était perdu tout droit
            robot.head.steer_center()
            robot.motor.drive(Direction.BACKWARD, SPEED_TURNING_PCT, fast_accel=True)

    else:
        # Out of the manuver when detecting the line
        with robot.state.lock:
            robot.state.already_lost = False
            robot.state.maneuver = False
            robot.state.post_time = time.time()
            if robot.state.last_turn != 0:
                robot.state.post_manuver = True
            else:
                robot.state.post_manuver = False


def thread_controller(robot: Robot, interval: float) -> None:
    """
    Le "cerveau" du robot. Boucle de décision principale :
    Analyse les informations des capteurs et donne les ordres aux moteurs de direction et de propulsion.
    """
    log = logger.get_logger("CTRL")
    log.info("Thread démarré (intervalle=%.3f s)", interval)

    last_action: Optional[LinePosition] = None

    while True:
        # ── 1. Lecture de l'état actuel ───────────────────────────
        with robot.state.lock:
            if not robot.state.running:
                break
            emergency = robot.state.emergency_stop
            action = robot.state.line_action
            maneuver = robot.state.maneuver

        # ── 2. Gestion de l'urgence (Priorité Absolue) ────────────
        if emergency:
            robot.motor.stop()
            robot.head.steer_center()
            log.warning("⚠ OBSTACLE détecté — arrêt d'urgence")
            time.sleep(interval)
            continue  # Saute le reste de la boucle tant que l'obstacle est là

        # ── 3. Affichage des changements de comportement ──────────
        if action != last_action:
            log.info("Changement de comportement → %s", action.name)
            if action == LinePosition.LINE_LOST:
                log.warning("Ligne perdue — recherche active / attente…")
            elif action == LinePosition.INTERSECTION:
                log.info("Intersection détectée — passage tout droit")
            last_action = action

        # ── 4. Logique de navigation et de récupération ───────────

        # === MODE A : Conduite normale (pas en pleine manœuvre de recul) ===
        if not maneuver:
            # --- ÉTAPE POST-MANŒUVRE ---
            if robot.state.post_manuver:
                movement_post_manuver(robot, interval, log)

            # --- CONDUITE NORMALE DE BASE ---
            else:
                basic_movement(robot, interval, log)

        # === MODE B : En pleine manœuvre de récupération (recul) ===
        else:
            movement_manuver(robot, interval, log)

        time.sleep(interval)

    # ── Arrêt propre en fin de thread (quand robot.state.running devient False) ──
    robot.motor.stop()
    robot.head.steer_center()
    log.info("Thread arrêté")


def thread_buzzer(robot: Robot) -> None:
    """
    Gère les effets sonores du robot.
    - Obstacle (emergency_stop)         -> sirène POLICE
    - Manœuvre de récupération          -> son défini par LINE_LOST_SOUND
    - Robot en mouvement normal         -> thème MII
    - À l'arrêt                         -> silence
    """
    log = logger.get_logger("BUZZER")
    log.info("Thread démarré")

    # Fonctions de vérification d'état (callbacks) pour que la fonction play()
    # sache quand elle doit arrêter le son en cours de lecture.
    def emergency_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.emergency_stop

    def maneuver_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.maneuver and not robot.state.emergency_stop

    def driving_active() -> bool:
        with robot.state.lock:
            return robot.state.running and robot.state.driving and not robot.state.emergency_stop

    # ATTENTION: La boucle est actuellement sur `while False:`
    # Ce thread est donc virtuellement inactif à l'exécution.
    while False:
        with robot.state.lock:
            running = robot.state.running
            emergency = robot.state.emergency_stop
            maneuver = robot.state.maneuver
            driving = robot.state.driving

        if not running:
            break

        if emergency:
            play(POLICE, emergency_active)
        elif maneuver and LINE_LOST_SOUND == "POLICE":
            play(POLICE, maneuver_active)
        elif maneuver and LINE_LOST_SOUND == "MII":
            play(MII, maneuver_active)
        elif driving:
            play(MII, driving_active)
        else:
            time.sleep(SENSOR_INTERVAL_S)

    log.info("Thread arrêté")