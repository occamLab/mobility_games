from audiolazy import *
import numpy as np
import time

def freq(name):
    """ Generates frequency based on note name.
    Example: freq("G4")"""

    try:
        letter = name[0]
        name = name[1:]
        mod = False
        if name[0] in ("#", "b"):
            letter += name[0]
            name = name[1:]
        octv = int(name)

        freqs = {"C" : 16.35, "C#" : 17.32, "Db" : 17.32, "D" : 18.35,
                "D#" : 19.45, "Eb" : 19.45, "E" : 20.60, "E#" : 21.83, "Fb" : 20.60,
                "F" : 21.83, "F#" : 23.12, "Gb" : 23.12, "G" : 24.50, "G#" : 25.96,
                "Ab" : 25.96, "A" : 27.50, "A#" : 29.14, "Bb" : 29.14, "B" : 30.87,
                "B#" : 32.70, "Cb" : 15.49}

        return freqs[letter] * (2 ** octv)

    except:
        print "That is not a note."

def delay(sig, delaynum, delayinterval, startamp):
  """ Simple feedforward delay effect """
  smix = Streamix()
  sig = thub(sig, delaynum) # Auto-copy 3 times (remove this line if using feedback)
  smix.add(0, sig)
  # To get a feedback delay, use "smix.copy()" below instead of both "sig"
  for i in range(1, delaynum):
      #print .3-(i/20)
      smix.add(delayinterval* ms, startamp*(delaynum-i)/delaynum * sig)
  #smix.add(300 * ms, .3 * sig) # You can also try other constants
  #smix.add(360 * ms, .125 * sig)
  return smix
  # When using the feedback delay proposed by the comments, you can use:
  #return smix.limit((1 + sum(dur for n, dur in notes)) * quarter_dur)
  # or something alike (e.g. ensuring that duration outside of this
# function), helping you to avoid an endless signal.

def arrays_to_sound(list_of_tracks, quarter_time):
    """ Takes in a list of numpy arrays for audio tracks and
    the time of a quarter note, in seconds.

    Returns a list of sound objects corresponding to the tracks."""
    i = 0
    sounds = []
    master_array = np.concatenate(list_of_tracks, 1)
    for chord in master_array:
        chord_list = []
        for note in chord:
            chord_list.append(pns([[note]], quarter_time)[0])
            i += 1
        sounds.append(chord_list)
    return np.asarray(sounds)

def synth(freq):
    """ Generates a karplus_strong sound at a given frequency."""
    rate = 44100 # Sampling rate, in samples/second
    s, Hz = sHz(rate) # Seconds and hertz
    ms = 1e-3 * s
    return karplus_strong(freq * Hz)

def player():
    """ Generates the player object."""
    return AudioIO(True)

def play(sound, player):
    """ Plays a sound object """
    rate = 44100
    player.play(sound, rate = rate)

def pns(list_of_chords, t = 0.5, beat = 0):
    """ Plays notes from a numpy array, where notes along the first dimension
    are played sequentially and notes along the second dimension are played
    simultaneously. "R" or "r" can be used to indicate rest.

    t is the length of each note, in seconds.

    Example: pns(np.asarray([["C4", "E4, "G4"], ["r, r, r"]["C4, "F4, "A4"]]))"""
    if beat > 0:
        list_of_chords = list_of_chords[beat - 1, :]
    rate = 44100
    s, Hz = sHz(rate)
    n = 0
    b = []
    for chord in list_of_chords:
        a = []
        for note in chord:
            if note.lower() == "r" or note == ".":
                pass
            else:
                a.append(synth(freq(note)))
        sample = zeros(int(t*s*n)).append(sum(a) * 0.5)
        n += 1
        b.append(sample)
    delay_time = (t * n)
    n += 4
    sound = sum(b).take(int(t*s*n))
    return sound, delay_time

def song_sample_1():
    track_1 = np.asarray(["C3 . . . G3 . . .".split()]).T
    track_2 = np.asarray(["Eb4 . F4 Eb4 D4 . F4 Eb4".split()]).T
    track_3 = np.asarray(["C5 G5 C6 G5 B6 D6 B6 G5".split()]).T
    return [track_1, track_2, track_3]

if __name__ == '__main__':
    print("Several functions to assist with realtime audio generation.")
