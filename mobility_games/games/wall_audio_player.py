#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
import mobility_games.auditory.audio_controller as ac
from os import system
import random

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
        self.rate = 1.0/20.0;
        self.randrange = 1;
        self.r = None
    def get_dist(self, msg):
        self.dist = msg.data
        filtereddist =  max(min(3*self.dist, 20), 2)
        self.rate = 1.0/filtereddist;
        self.r = rospy.Rate(1.0/self.rate)
    def run(self):
        self.r = rospy.Rate(1.0/self.rate) #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        #self.dist = 0
        while not rospy.is_shutdown(): #Start main while loop
            #self.dist = random.random()*4
            if not self.dist is None:
                #if rospy.Time.now()-self.last_sound_time > rospy.Duration(.01) and self.walldist is not 0: #If it's been longer than 2 seconds since last sound, and the wall distance is greater than half a meter
                self.last_sound_time = rospy.Time.now() #reset sound time
                if abs(self.dist) < self.altsounddist and self.rmode == 'ding':
                    system("aplay ../auditory/sound_files/ding.wav")
                else:
                    freq = ac.freq('A4')
                    vol = .15
                    music = None
                    seconds = 5*self.rate
                    fadeinendtime = 1.0*self.rate
                    fadeoutendtime = 1.0*self.rate
                    fadeintime = .2*self.rate
                    fadeouttime = .2*self.rate
                    print("rate: " + str(self.rate))
                    if ("p" in self.gmode):#self.gmode.contains('p')):
                        #print("wowowowow")
                        freq = max(min(100*math.exp((abs(self.dist+random.random()*self.randrange)-self.altsounddist)/1.6)+65.4*2, 2092.8), 65.4*2)*2 #set frequency based on distance
                        if self.dist < .7:
                            freq = ac.quantize(freq, quantizetype = "M", key="C")
                        elif self.dist < 1.4:
                            freq = ac.quantize(freq, quantizetype = "dom7", key="G")
                        else:
                            freq = ac.quantize(freq, quantizetype = "m7", key="D")
                        #print (freq)
                    elif ("v" in self.gmode):
                        vol = (.8/(1+math.exp((self.dist)/1.5-1))+.2)*vol#insert volume function here. that goes from 1 to 0 with increasing distance to wall.
                    elif ("t" in self.gmode):
                        #Do Music Tracks based on distance, further away = less music
                        pass
                    else:
                        print('INVALID AUDIO MODE')
                        break
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth

                    synth = ac.synth(freq, synth="sin", fadebool=False) #create synth for sound
                    synth = ac.fade_out(ac.fade_in(synth, fadeinendtime, fadeintime), fadeoutendtime, fadeouttime)
                    if ("p" in self.gmode or "v" in self.gmode):
                        ac.playstream(synth, self.player, seconds = seconds, volume = vol)#synth, self.player, seconds = .5, volume = vol)
                    elif ("t" in self.gmode):
                        pass
            print(self.dist)
            self.r.sleep() #wait until next iteration

if __name__ == '__main__': #Run Code
    node = wall_audio_player('pv', 'noding')
    node.run()
