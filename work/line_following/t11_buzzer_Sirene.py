#!/usr/bin/env python3
from gpiozero import TonalBuzzer
from time import sleep

# Initialize a TonalBuzzer connected to GPIO18 (BCM)
tb = TonalBuzzer(18)

# Define a musical tune as a sequence of notes and durations.
POLICE =	[
    ["A5", 0.4], ["D5", 0.4],
    ["A5", 0.4], ["D5", 0.4],
]
MII = [
    # 1ère phrase : tut, tut, tut, tut, tut... tut tut tuuut !
    ["F#4", 0.25],[None, 0.25], ["A4", 0.25], ["C#5", 0.25], [None, 0.25],
    ["A4", 0.25],[None, 0.25],  ["F#4", 0.25],

    ["D4", 0.15],[None, 0.1],["D4", 0.15],
    [None, 0.1],["D4", 0.15],[None, 0.1],
    [None, 1],["C#4", 0.25],

    # 2ème phrase
    ["D4", 0.25], ["F#4", 0.25], ["A4", 0.25],
    ["C#5", 0.125],[None, 0.125], [None, 0.25], ["A4", 0.125],[None, 0.125], ["F#4", 0.25],
    ["E5", 0.750],["D#5", 0.25],["D5", 0.5],
    [None, 0.5],

    # 3ème phrase (la grande descente)
    ["G#4", 0.5], ["C#5", 0.25], ["F#5", 0.25],
    ["C#5", 0.125],[None, 0.375], ["G#4", 0.125],[None, 0.375],
    ["C#5", 0.125],[None, 0.375],  ["G4", 0.25],
    ["F#4", 0.25],[None, 0.25],["E4", 0.125], [None, 0.375],
    ["C4", 0.125],[None, 0.125], ["C4", 0.125],[None, 0.125],
    ["C4", 0.125], [None, 0.125],
    ["C4", 0.125], [None, 0.875],
    ["C4", 0.125],[None, 0.125],
    ["C4", 0.125], [None, 0.125],
    ["C4", 0.125], [None, 0.875],
    ["D#4", 0.5], ["D4", 0.5],

    ["C#4", 0.25], [None, 0.25], ["A4", 0.25], ["C#5", 0.25], [None, 0.25],
    ["A4", 0.25], [None, 0.25], ["F#4", 0.25], ["E4", 0.15], [None, 0.1], ["E4", 0.15],
    [None, 0.1], ["E4", 0.15], [None, 0.35], ["G#5", 0.15], [None, 0.1], ["G#5", 0.15],
    [None, 0.1], ["G#5", 0.15], [None, 1.125],
    ["D4", 0.25], ["F#4", 0.25], ["A4", 0.25],
    ["C#5", 0.125], [None, 0.125], [None, 0.5], ["A4", 0.125], [None, 0.125], ["F#4", 0.25],
    ["C#5", 1], ["B5", 0.5],
    [None, 0.5],

    ["B5", 0.25], ["G4", 0.25], ["D4", 0.25], ["C#4", 0.5],
    ["B5", 0.25], ["G4", 0.25], ["D4", 0.25],
    ["A5", 0.25], ["F#4", 0.25], ["C4", 0.25], ["B5", 0.5],
    ["F4", 0.25], ["D4", 0.25], ["B5", 0.25],
    ["E4", 0.125],[None, 0.125],
    ["E4", 0.125], [None, 0.125],
    ["E4", 0.125], [None, 1.125],
    ["A5", 0.25],["B5", 0.25],["C#5", 0.25],["D5", 0.25],["F#5", 0.25],["A6", 0.5],[None, 1.5],

    ["A4", 0.5],["A#4", 0.5],
    ["B5", 0.75],["A#4", 0.25],["B5", 1.5],
    ["A4", 0.25],["A#4", 0.25],["B5", 0.25],["F#4",0.5],["C#4", 0.25],
    ["B5", 0.75],["A#4", 0.25],["B5",1.5],[None,0.5],["B5", 0.5],["C4", 0.5],

    ["C#4", 0.75],["C5", 0.25],["C#4", 1.5],
    ["C#4", 0.25],["C4", 0.25],["C#4", 0.25],["G#4",0.5],["D#4", 0.25],
    ["C#4", 0.75],["D#4", 0.25],["B5", 0.75],["C#4", 0.25],["D4",0.25],["A5", 0.25],[None, 0.25],
    ["D4", 0.25],["G#4", 0.125],[None, 0.125],
    ["G#4", 0.125], [None, 0.125],
    ["G#4", 0.125], [None, 0.875],


]

def play(tune, should_continue=lambda: True):
    """
    Play a musical tune using the buzzer.
    :param tune: List of tuples (note, duration),
    where each tuple represents a note and its duration.
    :param should_continue: callable réévalué avant chaque note ; dès qu'il
    renvoie False, la lecture s'arrête immédiatement (au lieu de jouer
    l'air en entier).
    """
    for note, duration in tune:
        if not should_continue():
            break
        tb.play(note)  # Play the note on the buzzer
        sleep(float(duration))  # Delay for the duration of the note
    tb.stop()  # Stop playing after the tune is complete

def close_buzzer():
    tb.close()

if __name__ == "__main__":
    try:
        play(MII)  # Execute the play function to start playing the tune.

    except KeyboardInterrupt:
        # Handle KeyboardInterrupt for graceful termination
        pass
