import argparse
from time import sleep
from gpiozero import DistanceSensor

import logging.logger as log

# ── Constantes ──────────────────────────────────────────────────
PIN_ULTRASONIC_TRIGGER = 23
PIN_ULTRASONIC_ECHO    = 24

class DistanceColor:
    """Codes ANSI pour colorer les sorties texte dans le terminal."""
    NEAR = "\033[31m"
    OK   = "\033[33m"
    FAR  = "\033[32m"
    END  = "\033[0m"


#  CAPTEUR ULTRASON
class UltrasonicSensor:
    """
    Capteur de distance HC-SR04 via gpiozero.DistanceSensor.
    Retourne la distance en millimètres.
    """

    def __init__(self,
                 trigger_pin: int   = PIN_ULTRASONIC_TRIGGER,
                 echo_pin:    int   = PIN_ULTRASONIC_ECHO,
                 max_dist_m:  float = 2.0):
        self._log = log.get_logger("US_HW")
        self._device = DistanceSensor(
            echo=echo_pin,
            trigger=trigger_pin,
            max_distance=max_dist_m
        )
        self.max_distance_mm = max_dist_m * 1000
        self._log.info("Capteur ultrason initialisé (trigger=%d, echo=%d)", trigger_pin, echo_pin)

    def read_mm(self) -> float:
        """Retourne la distance mesurée en millimètres."""
        return self._device.distance * 1000


def parse_arguments():
    """Gère l'analyse des arguments de la ligne de commande.
    Retourne:
        argparse.Namespace: Les arguments de la ligne de commande analysés.
    """
    parser = argparse.ArgumentParser(
        description="Calcule la position d'un objet par rapport au robot à l'aide d'un capteur ultrason."
    )
    parser.add_argument('--trigger', type=int, default=23, help="Broche GPIO pour le Trigger (par défaut : 23)")
    parser.add_argument('--echo', type=int, default=24, help="Broche GPIO pour l'Echo (par défaut : 24)")
    parser.add_argument('--max-dist', type=float, default=2.0,
                        help="Distance maximale de détection en mètres (par défaut : 2.0)")
    parser.add_argument('--level1', type=float, default=70.0,
                        help="Seuil du niveau 1 (Alerte rouge) en mm (par défaut : 70.0)")
    parser.add_argument('--level2', type=float, default=100.0,
                        help="Seuil du niveau 2 (Alerte jaune) en mm (par défaut : 100.0)")
    parser.add_argument('--interval', type=float, default=0.05, help="Intervalle de mesure en secondes (par défaut : 0.05)")

    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    sensor = UltrasonicSensor(
        trigger_pin=args.trigger,
        echo_pin=args.echo,
        max_dist_m=args.max_dist
    )

    # Boucle principale
    try:
        print("Démarrage des mesures en continu... (Appuyez sur Ctrl+C pour arrêter)")
        while True:
            distance = sensor.read_mm()

            if distance > sensor.max_distance_mm:
                print(
                    f"{DistanceColor.FAR}Aucun objet détecté dans un rayon de {sensor.max_distance_mm:.2f} mm{DistanceColor.END}")
            elif distance > args.level2:
                print(f"{DistanceColor.FAR}Objet situé à : {distance:.2f} mm{DistanceColor.END}")
            elif distance > args.level1:
                print(f"{DistanceColor.OK}Objet situé à : {distance:.2f} mm{DistanceColor.END}")
            else:
                print(f"{DistanceColor.NEAR}Objet situé à : {distance:.2f} mm{DistanceColor.END}")

            sleep(args.interval)

    except KeyboardInterrupt:
        print("\nProgramme interrompu. Au revoir !")
        print("Programme développé par l'Équipe C - MasterCamp SE 2026.")