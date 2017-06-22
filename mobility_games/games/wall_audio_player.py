#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import math
import mobility_games.auditory.audio_controller as ac
from os import system, path
import random
from dynamic_reconfigure.server import Server
from mobility_games.cfg import WallAudioConfig
from rospkg import RosPack
import mobility_games.auditory.play_wav as pw

class wall_audio_player(object):
    def __init__(self):
        top = RosPack().get_path("mobility_games")
        rospy.init_node('smart_wall_audio')
        rospy.Subscriber('/smart_wall_dist', Float64, self.get_dist)
        self.dingsound = path.join(top, 'auditory/sound_files/ding.wav')
        srv = Server(WallAudioConfig, self.config_callback)
        #self.gmode = rospy.get_param('~pc_audio_gmode', 'pvr')
        self.pitch = rospy.get_param('~pc_pitch', True)
        self.vol = rospy.get_param('~pc_vol', True)
        self.speed = rospy.get_param('~pc_rate', True)
        self.track = rospy.get_param('~pc_track', False)
        self.ding = rospy.get_param('~pc_ding', False)
        self.randrange = rospy.get_param('~pc_randrange', 1);
        self.quantizetype = rospy.get_param('~pc_quanttype', 'dom7')
        self.key = rospy.get_param('~pc_key', 'A')
        self.last_sound_time = rospy.Time.now() #Initialize the last time a sound was made
        self.dist = None
        self.player = ac.player() #play the sound.
        self.altsounddist = .5;
        self.rate = 1.0/20.0;
        self.r = rospy.Rate(1.0/self.rate);

    def config_callback(self, config, level):
        self.pitch = config['pc_pitch']
        self.vol = config['pc_vol']
        self.speed = config['pc_rate']
        self.track = config['pc_track']
        self.ding = config['pc_ding']
        self.quantizetype = config['pc_quanttype']
        self.key = config['pc_key']
        self.randrange = config['pc_randrange']
        return config

    def get_dist(self, msg):
        self.dist = msg.data
        if self.speed:
            filtereddist =  max(min(3*self.dist, 20), 2)
            self.rate = 1.0/filtereddist;
            self.r = rospy.Rate(1.0/self.rate)
        else:
            pass
    def run(self):
        #self.r =  #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        #self.dist = 0
        rewardsound=pw.Wav(self.dingsound)
        while not rospy.is_shutdown(): #Start main while loop
            #self.dist = random.random()*4
            if not self.dist is None:
                #if rospy.Time.now()-self.last_sound_time > rospy.Duration(.01) and self.walldist is not 0: #If it's been longer than 2 seconds since last sound, and the wall distance is greater than half a meter
                self.last_sound_time = rospy.Time.now() #reset sound time
                if abs(self.dist) < self.altsounddist and self.ding:
                    rewardsound.close()
                    rewardsound=pw.Wav(self.dingsound)
                    rewardsound.play()

                else:
                    freq = ac.freq('C5')
                    vol = .8
                    music = None
                    seconds = 5*self.rate
                    fadeinendtime = 1.0*self.rate
                    fadeoutendtime = 1.0*self.rate
                    fadeintime = .2*self.rate
                    fadeouttime = .2*self.rate
                    #print("rate: " + str(self.rate))
                    #if (self.track):
                        #Do Music Tracks based on distance, further away = less music
                    #    pass
                    if (self.pitch):#self.gmode.contains('p')):
                        #print("wowowowow")
                        freq = max(min(100*math.exp((abs(self.dist+random.random()*self.randrange)-self.altsounddist)/1.6)+65.4*2, 2092.8), 65.4*2)*2 #set frequency based on distance
                        #if self.dist < .7:
                        #    freq = ac.quantize(freq, quantizetype = "M", key="C")
                        #elif self.dist < 1.4:
                        #    freq = ac.quantize(freq, quantizetype = "dom7", key="G")
                        #else:
                        freq = ac.quantize(freq, quantizetype = self.quantizetype, key=self.key)
                        #print (freq)
                    if (self.vol):
                        vol = (.8/(1+math.exp((self.dist)/1.5-1))+.2)*vol#insert volume function here. that goes from 1 to 0 with increasing distance to wall.
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth

                    if self.track:
                        pass
                    else:
                        synth = ac.synth(freq, synth="sin", fadebool=False) #create synth for sound
                        synth = ac.fade_out(ac.fade_in(synth, fadeinendtime, fadeintime), fadeoutendtime, fadeouttime)
                        ac.playstream(synth, self.player, seconds = seconds, volume = vol)#synth, self.player, seconds = .5, volume = vol)
            #print(walldistself.dist)
            self.r.sleep() #wait until next iteration

if __name__ == '__main__': #Run Code
    node = wall_audio_player()
    node.run()
