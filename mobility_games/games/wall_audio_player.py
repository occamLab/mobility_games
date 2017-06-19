#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
import mobility_games.auditory.audio_controller as ac
from os import system

class wall_audio_player(object):
    def __init__(self):
        rospy.init_node('smart_wall_audio')
        rospy.Subscriber('/smart_wall_dist', Float64, self.get_dist)
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
                if abs(self.dist) < self.altsounddist:
                    system("aplay ../auditory/sound_files/ding.wav")
                else:
                    freq = max(min(100*math.exp((abs(self.dist)-self.altsounddist)/1.6)+65.4, 2092.8), 65.4) #set frequency based on distance
                    freq = ac.quantize(freq, key="C")
                    self.synth = ac.synth(freq, synth="digitar", fade = 1) #create synth for sound
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth
                    ac.playstream(self.synth, self.player, seconds = .7)
            r.sleep() #wait until next iteration

if __name__ == '__main__': #Run Code
    node = wall_audio_player()
    node.run()
