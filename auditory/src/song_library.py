import numpy as np
import random

def song_sample_1():
    """Short song sample. Recommended tempo: 240 bpm."""

    track_0 = silence(8)
    track_1 = np.asarray(["C3 . . . G3 . . .".split()]).T
    track_2 = np.asarray(["Eb4 . F4 Eb4 D4 . F4 Eb4".split()]).T
    track_3 = np.asarray(["C5 G5 C6 G5 D5 G5 D6 ".split()]).T
    return [track_0, track_1, track_2, track_3]

def mario():
    track_0 = silence(16)
    track_1 = np.asarray(["A4 A4 . A4 . F4 A4 . C5 . . . C4 . . .".split()]).T
    return [track_0, track_1]

def final_countdown():
    """ Sample from 'The Final Countdown.' Recommended tempo: 480 bpm."""

    measures = 12
    track_0 = silence(16 * measures)

    track_1 = np.asarray([(" A2 .  .  .  .  .  .  .  .  .  .  .  G2 .  .  . " +
                        "    F2 .  .  .  .  .  .  .  .  .  .  .  E2 .  .  . " +
                        "    D2 .  .  .  .  .  .  .  .  .  .  .  D2 .  .  . " +
                        "    G2 .  .  .  .  .  .  .  G#2 . .  .  .  .  .  . " +

                        "    A2 .  .  .  .  .  .  .  .  .  .  .  G2 .  .  . " +
                        "    F2 .  .  .  .  .  .  .  .  .  .  .  E2 .  .  . " +
                        "    D2 .  .  .  .  .  .  .  .  .  .  .  D2 .  .  . " +
                        "    G2 .  .  .  .  .  .  .  G#2 . .  .  .  .  .  . " +

                        "    A2 .  .  .  .  .  .  .  B2 .  .  .  .  .  .  . " +
                        "    C3 .  B2 .  A2 .  G2 .  F2 .  .  .  F2 .  .  . " +
                        "    E2 .  .  .  B2 .  .  .  B2 .  .  .  B2 .  .  . " +
                        "    E2 .  .  . G#2 .  .  . G#2 .  .  . G#2 .  .  .").split()]).T


    track_2 = np.asarray([(" A3 . " * 8                                        +
                        "    F3 . " * 8                                        +
                        "    D3 . " * 8                                        +
                        "    B3 . " * 8                                        +

                        "    A3 . " * 8                                        +
                        "    F3 . " * 8                                        +
                        "    D3 . " * 8                                        +
                        "    B2 . " * 8                                        +

                        "    C3 .  .  .  .  .  .  .  B2 .  .  .  .  .  .  . " +
                        "    G3 .  .  .  G3 .  .  .  F3 .  .  .  A3 .  .  . " +
                        "    A3 .  E4 .  E4 .  E4 .  E4 .  E4 .  E4 .  E4 . " +
                        "    G#3 . E4 .  E4 .  E4 .  E4 .  E4 .  E4 .  E4 . ").split()]).T

    track_3 = np.asarray([(" C5 .  .  .  C4  .  E5 D5 E5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  F5 E5 F5 .  E5 .  D5 .  .  . " +
                        "    .  .  .  .  .  .  F5 E5 F5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  D5 C5 D5 .  C5 .  B4 .  D5 . " +

                        "    C5 .  .  .  .  .  E5 D5 E5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  F5 E5 F5 .  E5 .  D5 .  .  . " +
                        "    .  .  .  .  .  .  F5 E5 F5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  D5 C5 D5 .  C5 .  B4 .  D5 . " +

                        "    C5 .  .  .  .  .  B4 C5 D5 .  .  .  .  .  C5 D5" +
                        "    E5 .  D5 .  C5 .  B4 .  A4 .  .  .  F5 .  .  . " +
                        "    E5 .  .  .  .  .  .  .  .  .  .  .  E5 F5 E5 D5" +
                        "    E5 .  .  .  A3 .  A3 .  C4 .  .  .  B3 .  .  . ").split()]).T

    track_4 = np.asarray([(" B4 .  .  .  A4 .  A4 .  C5 .  .  .  E4 .  .  . " +
                        "    .  .  .  .  .  .  C5 .  C5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  D5 .  D5 .  .  .  F4 .  .  . " +
                        "    .  .  .  .  .  .  B4 .  B4 .  .  .  G4 .  .  . " +

                        "    C4 .  .  .  .  .  A4 .  C5 .  .  .  E4 .  .  . " +
                        "    .  .  .  .  .  .  C5 .  C5 .  .  .  A4 .  .  . " +
                        "    .  .  .  .  .  .  D5 .  D5 .  .  .  F4 .  .  . " +
                        "    .  .  .  .  .  .  B4 .  B4 .  .  .  G4 .  .  . " +

                        "    A4 .  .  .  .  .  G4 .  .  .  .  .  .  .  .  . " +
                        "    C5 .  D5 .  C5 .  B4 .  F4 .  .  .  C5 .  .  . " +
                        "    B4 .  .  .  .  .  .  .  .  .  .  .  .  .  .  . " +
                        "    B4 .  .  .  A4 .  A4 .  C5 .  .  .  B4 .  .  . ").split()]).T

    track_5 = np.asarray([(" E6 C6 A5 C6 E6 C6 A5 C6 E6 C6 A5 C6 E6 C6 A5 C6" +
                        "    F6 C6 A5 C6 F6 C6 A5 C6 F6 C6 A5 C6 F6 C6 A5 C6" +
                        "    F6 D6 A5 D6 F6 D6 A5 D6 F6 D6 A5 D6 F6 D6 A5 D6" +
                        "    G6 D6 B5 D6 G6 D6 B5 D6 G6 D6 B5 D6 G6 D6 B5 D6" +

                        "    E6 C6 A5 C6 E6 C6 A5 C6 E6 C6 A5 C6 E6 C6 A5 C6" +
                        "    F6 C6 A5 C6 F6 C6 A5 C6 F6 C6 A5 C6 F6 C6 A5 C6" +
                        "    F6 D6 A5 D6 F6 D6 A5 D6 F6 D6 A5 D6 F6 D6 A5 D6" +
                        "    G6 D6 B5 D6 G6 D6 B5 D6 G6 D6 B5 D6 G6 D6 B5 D6" +

                        "    .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  . " +
                        "    E6 .  D6 .  C6 .  B5 .  A5 .  .  .  F6 .  .  . " +
                        "    E6 B5 A5 B5 E6 B5 A5 B5 E6 B5 A5 B5 E6 B5 A5 B5" +
                        "    E6 B5 Ab5 B5 E6 B5 Ab5 B5 E6 B5 Ab5 B5 E6 B5 Ab5 B5").split()]).T

    print(len(track_0), len(track_1), len(track_2), len(track_3), len(track_4), len(track_5))
    return [track_0, track_1, track_2, track_3, track_4, track_5]

def silence(num_beats):
    rests = ". " * num_beats
    return np.asarray([rests.split()]).T
