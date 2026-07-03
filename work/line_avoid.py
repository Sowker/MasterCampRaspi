"""
Module de suivi de cercle (ligne noire sur fond blanc).
Logique inversée par rapport au suivi de ligne normal.

Convention : 1 = ligne noire détectée, 0 = sol clair.
But : rester à l'intérieur d'un cercle noir en ajustant la trajectoire.
"""

import argparse
import time
from enum import IntEnum
from gpiozero import InputDevice

import logger as log


class CirclePosition(IntEnum):
    """Actions déduites des capteurs pour rester dans le cercle."""
    STRAIGHT        = 0  # Ligne au milieu → tout droit
    TURN_LEFT_SOFT  = 1  # Ligne à droite → tourner doux à gauche
    TURN_LEFT_HARD  = 2  # Ligne à droite + milieu → tourner fort à gauche
    TURN_RIGHT_SOFT = 3  # Ligne à gauche → tourner doux à droite
    TURN_RIGHT_HARD = 4  # Ligne à gauche + milieu → tourner fort à droite
    INTERSECTION    = 5  # Toutes les lignes → ambiguïté
    LOST_IN_CENTER  = 6  # Aucune ligne → perdu au centre du cercle

    @property
    def label(self) -> str:
        translations = {
            CirclePosition.STRAIGHT:        "Tout droit ⬆️",
            CirclePosition.TURN_LEFT_SOFT:  "Tourner à gauche (léger) ↖️",
            CirclePosition.TURN_LEFT_HARD:  "Tourner à gauche (fort) ⬅️",
            CirclePosition.TURN_RIGHT_SOFT: "Tourner à droite (léger) ↗️",
            CirclePosition.TURN_RIGHT_HARD: "Tourner à droite (fort) ➡️",
            CirclePosition.INTERSECTION:    "Ambiguïté (tous capteurs) ➕",
            CirclePosition.LOST_IN_CENTER:  "Perdu au centre ❓",
        }
        return translations[self]


# ── Constantes ─────────────────────────────────────────────────

# with sensor reversed
PIN_LINE_LEFT = 17
PIN_LINE_MIDDLE = 27
PIN_LINE_RIGHT = 22


# ═══════════════════════════════════════════════════════════════════
#  CAPTEURS POUR SUIVI DE CERCLE
# ═══════════════════════════════════════════════════════════════════

class CircleTracker:
    """
    Barrette de 3 capteurs infrarouges optimisée pour rester dans un cercle.

    Logique :
    - Capteur DROIT seul → Tourner DOUX à gauche (ligne à droite)
    - Capteur DROIT + MILIEU → Tourner FORT à gauche (trop à droite)
    - Capteur MILIEU seul → Tout droit (ligne bien centrée)
    - Capteur GAUCHE seul → Tourner DOUX à droite (ligne à gauche)
    - Capteur GAUCHE + MILIEU → Tourner FORT à droite (trop à gauche)
    - Tous les capteurs → Ambiguïté
    - Aucun capteur → Perdu au centre du cercle
    """

    TRUTH_TABLE = {
        (0, 1, 0): CirclePosition.STRAIGHT,  # Milieu seul → tout droit
        (0, 0, 1): CirclePosition.TURN_RIGHT_SOFT,  # Droit seul → doux à gauche
        (0, 1, 1): CirclePosition.TURN_RIGHT_HARD,  # Droit + milieu → fort à gauche
        (1, 0, 0): CirclePosition.TURN_LEFT_SOFT,  # Gauche seul → doux à droite
        (1, 1, 0): CirclePosition.TURN_LEFT_HARD,  # Gauche + milieu → fort à droite
        (1, 1, 1): CirclePosition.INTERSECTION,  # Tous les trois → ambiguïté
        (0, 0, 0): CirclePosition.LOST_IN_CENTER,  # Aucun capteur → au centre
    }

    def __init__(self,
                 pin_left: int = PIN_LINE_LEFT,
                 pin_middle: int = PIN_LINE_MIDDLE,
                 pin_right: int = PIN_LINE_RIGHT):
        self._log = log.get_logger("CIRCLE_HW")
        self._left = InputDevice(pin=pin_left)
        self._middle = InputDevice(pin=pin_middle)
        self._right = InputDevice(pin=pin_right)
        self._log.info("Capteurs de cercle initialisés (L=%d, M=%d, R=%d)",
                       pin_left, pin_middle, pin_right)

    def read(self) -> tuple[int, int, int]:
        """Retourne l'état brut instantané (gauche, milieu, droite)."""
        return self._left.value, self._middle.value, self._right.value

    def read_action(self) -> CirclePosition:
        """
        Lit les capteurs matériels et renvoie directement l'action décodée
        pour le suivi de cercle.
        """
        left, middle, right = self.read()
        return self.decode(left, middle, right)

    @staticmethod
    def decode(left: int, middle: int, right: int) -> CirclePosition:
        """Traduit les 3 valeurs binaires en une action adaptée au cercle."""
        pattern = (left, middle, right)
        return CircleTracker.TRUTH_TABLE.get(pattern, CirclePosition.LOST_IN_CENTER)


def parse_arguments() -> argparse.Namespace:
    """Gère l'analyse des arguments pour le suivi de cercle."""
    parser = argparse.ArgumentParser(
        description="Suivi de cercle noir sur fond blanc pour robot MasterCamp."
    )
    parser.add_argument('--left', type=int, default=PIN_LINE_LEFT,
                        help=f"Broche GPIO Gauche (défaut : {PIN_LINE_LEFT})")
    parser.add_argument('--middle', type=int, default=PIN_LINE_MIDDLE,
                        help=f"Broche GPIO Milieu (défaut : {PIN_LINE_MIDDLE})")
    parser.add_argument('--right', type=int, default=PIN_LINE_RIGHT,
                        help=f"Broche GPIO Droite (défaut : {PIN_LINE_RIGHT})")
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    circle_tracker = CircleTracker(pin_left=args.left, pin_middle=args.middle, pin_right=args.right)

    print("Démarrage du système de suivi de cercle... Appuyez sur Ctrl+C pour arrêter.\n")

    try:
        while True:
            status_left, status_middle, status_right = circle_tracker.read()
            raw_data = f"(G:{status_left} M:{status_middle} D:{status_right})"
            action = circle_tracker.read_action()

            print(f"{raw_data:<14} -> {action.label}")

            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nProgramme interrompu. Au revoir !")
        print("Programme développé par l'Équipe C - MasterCamp SE 2026.")