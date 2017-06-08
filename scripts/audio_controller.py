from audiolazy import *

def freq(name):
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

def synth(freq):
    rate = 44100 # Sampling rate, in samples/second
    s, Hz = sHz(rate) # Seconds and hertz
    ms = 1e-3 * s
    return karplus_strong(freq)

if __name__ == '__main__':
    main()
    rate = 44100 # Sampling rate, in samples/second
    s, Hz = sHz(rate) # Seconds and hertz
    ms = 1e-3 * s
    start1 = 110
    start2 = 220
    multiplier = 2
    for i in range(5):
        note1 = karplus_strong(start1*pow(multiplier, i) * Hz) # Pluck "digitar" synth
        note2 = zeros(300 * ms).append(karplus_strong(start2 * pow(multiplier, i) * Hz))
        notes = (note1 + note2) * .5 #.5 is Amplitude
        sound = notes.take(int(1.4 * s)) # 2 seconds of a Karplus-Strong note
        with AudioIO(True) as player: # True means "wait for all sounds to stop"
          player.play(sound, rate=rate)
