from audiolazy import *
from audio_controller import *
import numpy as np
import time

if __name__ == '__main__':
    music = np.asarray([["A4"], ["C5"], ["E5"], ["A5"]])
    player = AudioIO(True)
    track1, delay = pns(music, 0.25)
    for i in range(0, 4):
        play(track1, player)
        time.sleep(delay)
