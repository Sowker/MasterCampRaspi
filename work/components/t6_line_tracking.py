import argparse
import time
from enum import IntEnum
from gpiozero import InputDevice

import logging.logger as log

class LinePosition(IntEnum):
    """Actions déduites des capteurs de ligne."""
    STRAIGHT        = 0
    TURN_LEFT_SOFT  = 1
    TURN_LEFT_HARD  = 2
    TURN_RIGHT_SOFT = 3
    TURN_RIGHT_HARD = 4
    INTERSECTION    = 5
    LINE_LOST       = 6

    @property
    def label(self) -> str:
        translations = {
            LinePosition.STRAIGHT:        "Tout droit ⬆️",
            LinePosition.TURN_LEFT_SOFT:  "Tourner à gauche (léger) ↖️",
            LinePosition.TURN_LEFT_HARD:  "Tourner à gauche (fort) ⬅️",
            LinePosition.TURN_RIGHT_SOFT: "Tourner à droite (léger) ↗️",
            LinePosition.TURN_RIGHT_HARD: "Tourner à droite (fort) ➡️",
            LinePosition.INTERSECTION:    "Intersection / Croisement ➕",
            LinePosition.LINE_LOST:       "Ligne perdue / En attente ❓",
        }
        return translations[self]


# ── Constantes ─────────────────────────────────────────────────

PIN_LINE_LEFT          = 22
PIN_LINE_MIDDLE        = 27
PIN_LINE_RIGHT         = 17


#  CAPTEURS DE LIGNE
class LineTracker:
    """
    Barrette de 3 capteurs infrarouges.
    Convention : 1 = ligne noire détectée, 0 = sol clair.
    """

    TRUTH_TABLE = {
        (0, 1, 0): LinePosition.STRAIGHT,
        (1, 1, 0): LinePosition.TURN_RIGHT_SOFT,
        (0, 1, 1): LinePosition.TURN_LEFT_SOFT,
        (1, 0, 0): LinePosition.TURN_RIGHT_HARD,
        (0, 0, 1): LinePosition.TURN_LEFT_HARD,
        (1, 1, 1): LinePosition.INTERSECTION,
        (0, 0, 0): LinePosition.LINE_LOST,
    }

    def __init__(self,
                 pin_left:   int = PIN_LINE_LEFT,
                 pin_middle: int = PIN_LINE_MIDDLE,
                 pin_right:  int = PIN_LINE_RIGHT):
        self._log    = log.get_logger("LINE_HW")
        self._left   = InputDevice(pin=pin_left)
        self._middle = InputDevice(pin=pin_middle)
        self._right  = InputDevice(pin=pin_right)
        self._log.info("Capteurs de ligne initialisés (L=%d, M=%d, R=%d)",
                       pin_left, pin_middle, pin_right)

    def read(self) -> tuple[int, int, int]:
        """Retourne l'état brut instantané (gauche, milieu, droite)."""
        return self._left.value, self._middle.value, self._right.value

    def read_action(self) -> LinePosition:
        """
        Méthode d'instance pratique.
        Lit les capteurs matériels et renvoie directement l'action décodée.
        """
        left, middle, right = self.read()
        return self.decode(left, middle, right)

    @staticmethod
    def decode(left: int, middle: int, right: int) -> LinePosition:
        """Traduit les 3 valeurs binaires en une action de conduite."""
        pattern = (left, middle, right)
        return LineTracker.TRUTH_TABLE.get(pattern, LinePosition.LINE_LOST)


def parse_arguments() -> argparse.Namespace:
    """Gère l'analyse des arguments de la ligne de commande pour le suivi de ligne."""
    parser = argparse.ArgumentParser(
        description="Script d'automatisation du suivi de ligne pour barrette de capteurs infrarouges (0 = Ligne noire)."
    )
    parser.add_argument('--left', type=int, default=PIN_LINE_LEFT, help=f"Broche GPIO Gauche (défaut : {PIN_LINE_LEFT})")
    parser.add_argument('--middle', type=int, default=PIN_LINE_MIDDLE, help=f"Broche GPIO Milieu (défaut : {PIN_LINE_MIDDLE})")
    parser.add_argument('--right', type=int, default=PIN_LINE_RIGHT, help=f"Broche GPIO Droite (défaut : {PIN_LINE_RIGHT})")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    line_tracking = LineTracker(pin_left=args.left, pin_middle=args.middle, pin_right=args.right)

    print("Démarrage du système de suivi de ligne... Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            status_left, status_middle, status_right = line_tracking.read()
            raw_data = f"(G:{status_left} M:{status_middle} D:{status_right})"
            action = line_tracking.read_action()

            # Affichage propre en utilisant la propriété de l'Enum
            print(f"{raw_data:<14} -> {action.label}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgramme interrompu. Au revoir !")
        print("Programme développé par l'Équipe C - MasterCamp SE 2026.")