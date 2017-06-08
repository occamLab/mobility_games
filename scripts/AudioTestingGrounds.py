from audiolazy import *


rate = 44100 # Sampling rate, in samples/second
s, Hz = sHz(rate) # Seconds and hertz
ms = 1e-3 * s
start1 = 110
start2 = 220
multiplier = 2
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
for i in range(5):
    note1 = karplus_strong(start1*pow(multiplier, i) * Hz) # Pluck "digitar" synth
    note2 = zeros(1000 * ms).append(karplus_strong(start2 * pow(multiplier, i) * Hz))
    notes = (note1 + note2) * .5 #.5 is Amplitude
    sound = notes.take(int(2.5 * s)) # 2 seconds of a Karplus-Strong notes
    with AudioIO(True) as player: # True means "wait for all sounds to stop"
      player.play(sound, rate=rate)
