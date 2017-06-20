#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
import mobility_games.auditory.audio_controller as ac
from os import system

class wall_audio_player(object):
    def __init__(self, generalmode, rewardmode):
        rospy.init_node('smart_wall_audio')
        rospy.Subscriber('/smart_wall_dist', Float64, self.get_dist)
        self.gmode = generalmode
        self.rmode = rewardmode
        self.last_sound_time = rospy.Time.now() #Initialize the last time a sound was made
        self.dist = None
        self.player = ac.player() #play the sound.
        self.altsounddist = .5;
    def get_dist(self, msg):
        self.dist = msg.data
    def run(self):
        r = rospy.Rate(2) #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        while not rospy.is_shutdown(): #Start main while loop
            if not self.dist is None:
                #if rospy.Time.now()-self.last_sound_time > rospy.Duration(.01) and self.walldist is not 0: #If it's been longer than 2 seconds since last sound, and the wall distance is greater than half a meter
                self.last_sound_time = rospy.Time.now() #reset sound time
                if abs(self.dist) < self.altsounddist and self.rmode == 'ding':
                    system("aplay ../auditory/sound_files/ding.wav")
                else:
                    freq = ac.freq('A3')
                    synth = None
                    vol = 1
                    music = None
                    if (self.gmode == 'pitch'):
                        freq = max(min(100*math.exp((abs(self.dist)-self.altsounddist)/1.6)+65.4, 2092.8), 65.4) #set frequency based on distance
                        freq = ac.quantize(freq, key="A")
                        synth = ac.synth(freq, synth="digitar", fade = 1) #create synth for sound
                    elif (self.gmode == 'volume'):
                        #insert volume function here. that goes from 1 to 0 with increasing distance to wall.
                        pass
                    elif (self.gmode == 'tracks'):
                        #Do Music Tracks based on distance, further away = less music
                        pass
                    else:
                        print('INVALID AUDIO MODE')
                        break
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth
                    if (self.gmode == 'pitch' or self.gmode == 'volume'):
                        ac.playstream(synth, self.player, seconds = .7)
                    elif (self.gmode == 'tracks'):
                        pass
            r.sleep() #wait until next iteration

if __name__ == '__main__': #Run Code
    node = wall_audio_player('pitch', 'ding')
    node.run()
