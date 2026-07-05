import argparse

from components.t5_ultrasonic_sensor import PIN_ULTRASONIC_ECHO, PIN_ULTRASONIC_TRIGGER
from components.t6_line_tracking import PIN_LINE_LEFT, PIN_LINE_MIDDLE, PIN_LINE_RIGHT


# ── Contrôleur ─────────────────────────────────────────────────────
OBSTACLE_THRESHOLD_MM = 150.0  # mm — seuil d'arrêt d'urgence

# attention répété dans threads
CTRL_INTERVAL_S       = 0.05   # s — période du thread contrôleur
SENSOR_INTERVAL_S     = 0.05   # s — période des threads capteurs

# ═══════════════════════════════════════════════════════════════════
#  ARGUMENTS CLI
# ═══════════════════════════════════════════════════════════════════

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Robot Line Follower — Team C — MasterCamp SE 2026",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    g_us = p.add_argument_group("Capteur ultrason")
    g_us.add_argument("--us-trigger",   type=int,   default=PIN_ULTRASONIC_TRIGGER)
    g_us.add_argument("--us-echo",      type=int,   default=PIN_ULTRASONIC_ECHO)
    g_us.add_argument("--obstacle-mm",  type=float, default=OBSTACLE_THRESHOLD_MM)

    g_line = p.add_argument_group("Capteurs de ligne")
    g_line.add_argument("--line-left",  type=int, default=PIN_LINE_LEFT)
    g_line.add_argument("--line-mid",   type=int, default=PIN_LINE_MIDDLE)
    g_line.add_argument("--line-right", type=int, default=PIN_LINE_RIGHT)

    g_timing = p.add_argument_group("Timing")
    g_timing.add_argument("--ctrl-interval",   type=float, default=CTRL_INTERVAL_S)
    g_timing.add_argument("--sensor-interval", type=float, default=SENSOR_INTERVAL_S)

    p.add_argument("--debug", action="store_true", help="Active les logs DEBUG")

    return p.parse_args()