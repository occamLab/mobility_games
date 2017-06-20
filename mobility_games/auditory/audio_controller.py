from audiolazy import *
from song_library import *
import math
import numpy as np
import rospy
import time
import sys

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
        print("That is not a note.")

def quantize(pitch, quantizetype = "scale", key = "A"):
    basepitch = freq(key+"0")
    toppitch = freq(key+"1")
    octavediff = 0;
    quantizedict = {"chromatic":[16.35, 17.32, 18.35, 19.45, 20.60, 21.83, 23.12, 24.50, 25.96, 27.59, 29.14, 30.87, 32.70],
                    "scale":[16.35, 18.35, 20.60, 21.83, 24.50, 27.59, 30.87, 32.70],
                    "M":[16.35, 20.60, 24.50, 32.70],
                    "m":[16.35, 19.45, 24.50, 32.70],
                    "M7":[16.35, 20.60, 24.50, 30.87, 32.70]}
    freqlist = quantizedict.get(quantizetype)
    C0ToNote = basepitch/freq("C0")
    freqlist = [i*C0ToNote for i in freqlist]
    #print(freqlist)
    if pitch > toppitch:
        while (pitch > toppitch):
            pitch = pitch/2
            octavediff+=1
    elif pitch < basepitch:
        while (pitch < basepitch):
            pitch = pitch*2
            octavediff-=1
    closeness = 100
    #print(pitch)
    for i in freqlist:
        pdiff = abs(pitch - i)
        if pdiff < closeness:
            closestpitch = i
            closeness = pdiff
    #print(closestpitch)
    closestpitch = closestpitch * math.pow(2, octavediff)
    return closestpitch
    #print(closestpitch)

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

def arrays_to_sound(list_of_tracks, quarter_time, amps = []):
    """ Takes in a list of numpy arrays for audio tracks and
    the time of a quarter note, in seconds.

    Returns a list of sound objects corresponding to the tracks."""
    print("Converting numpy array to audio.")
    if amps == []:
        amps = len(list_of_tracks[0]) * [1]
    i = 0
    sounds = []
    print("Concatenating matrices.")
    master_array = np.concatenate(list_of_tracks, 1)
    print("Interpreting chords.")

    #   Makes a loading bar in terminal because this can take a while
    length = 20
    sys.stdout.write("[%s]" % ("-" * length))
    sys.stdout.flush()
    sys.stdout.write("\b" * (length+1))

    nc = 0.0
    percent_done = 0
    sys.stdout.write("#")
    for chord in master_array:
        i = 0
        chord_list = []
        #   converts from symbolic array to pylazy sound objects
        for note in chord:
            chord_list.append(pns([[note]], quarter_time, amp = amps[i])[0])
            i += 1
        sounds.append(chord_list)
        old_p = percent_done
        percent_done = round((nc/len(master_array) * 100), 1)
        # Add to loading bar every time perent done increases significantly
        sys.stdout.write("#" * (int(percent_done/100*length) - int(old_p/100*length)))
        sys.stdout.flush()
        nc += 1.0
    sys.stdout.write("\n")
    print("Sounds generated.")
    return np.asarray(sounds)

def synth(freq, synth = "sin", fade = 0.4):
    """ Generates a karplus_strong sound at a given frequency."""
    rate = 44100 # Sampling rate, in samples/second
    s, Hz = sHz(rate) # Seconds and hertz
    ms = 1e-3 * s
    if synth == "sin":
        sound = saw_table(freq * Hz) * fadeout(fade*s)
    elif synth == "digitar":
        sound = karplus_strong(freq*Hz) * fadeout(fade*s)
    else:
        sound = karplus_strong(freq*Hz) * fadeout(fade*s)
    sound.append(zeros(Hz*(1 - fade)))
    return sound

def player():
    """ Generates the player object."""
    return AudioIO(True)

def play(sound, player):
    """ Plays a sound object """
    rate = 44100
    player.play(sound, rate = rate)

def playstream(stream, player, seconds = 1, volume = 1):
    """ Plays a stream on player for seconds seconds of stream """
    rate = 44100
    s, Hz = sHz(rate)
    ms = 1e-3*s
    sound = stream.take(int(seconds*s))
    sould = sound * volume
    player.play(sound, rate = rate)

def pns(list_of_chords, t = 0.5, beat = 0, amp = 1):
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
                #   Don't render audio if the current note is a rest
                pass
            else:
                #   Added extra time at the end of each note to avoid popping noise
                a.append(synth(freq(note)).append(zeros(s*t*8)))
        sample = zeros(int(t*s*n)).append(sum(a) * 0.05 * amp)
        n += 1
        b.append(sample)
    delay_time = (t * n)
    n += 4
    sound = sum(b).take(int(t*s*n))
    return sound, delay_time

def test_song(list_of_tracks, tempo, amps = [], start_num = 1, increase = 1):
    """ Plays a numpy array as a song.

    Two parameters:
    list_of_tracks - list of numpy arrays for each track of a song (see formats in song_library.py)
    tempo - tempo in beats per minute

    Three optional parameters:
    amps - list of float amplitudes to play song at. Should be a list of length equal to
            the number of tracks, where full volume is 1.0
    start_num - number of tracks to start out playing simultaneously (default one).
    increase - number of tracks to add every loop (default one). The song will continue to loop
            even after all tracks have been added.

    Tracks will be added in the order they appear in list_of_tracks. """

    if amps == []:
        amps = len(list_of_tracks[0]) * [1]
    quarter = 60.0/tempo
    beat = 1
    p = player()
    track_length = len(list_of_tracks[0])
    music = arrays_to_sound(list_of_tracks, quarter, amps = amps)
    num_tracks = start_num
    print("Sample playing.")
    while True:
        tic = time.clock()
        chord = sum(music[beat - 1, :num_tracks])
        play(chord, p)
        beat += 1
        if beat > track_length:
            beat = 1
            if num_tracks < len(list_of_tracks):
                num_tracks += increase
                if num_tracks > len(list_of_tracks):
                    num_tracks = len(list_of_tracks)
        dif = time.clock() - tic
        if dif < time.ctime(quarter):
            time.sleep(quarter - dif)

if __name__ == '__main__':
    test_song(final_countdown(), 480, amps = [0.8, 1.0, 0.8, 1.2, 1.0, 0.6], start_num = 3)
