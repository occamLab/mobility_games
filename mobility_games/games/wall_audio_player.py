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
        self.pitch = rospy.get_param('~pitch', True)
        self.pitchdefault = rospy.get_param('~pitchDefault', 'A4')
        self.vol = rospy.get_param('~vol', True)
        self.voldefault = rospy.get_param('~volDefault', .8)
        self.speed = rospy.get_param('~rate', True)
        self.speeddefault = rospy.get_param('~rateDefault', 20);
        self.track = rospy.get_param('~track', False)
        self.ding = rospy.get_param('~ding', False)
        self.randrange = rospy.get_param('~randRange', 0);
        self.quantizetype = rospy.get_param('~quantType', 'M')
        self.key = rospy.get_param('~key', 'A')
        self.on = rospy.get_param('~on', False)
        self.altsounddist = rospy.get_param('~successZoneSize', .5)
        self.maxfreqdist = 5.0
        self.maxfreq = 2092.8
        self.minfreq = 130.8
        self.reverse = False;
        self.last_sound_time = rospy.Time.now() #Initialize the last time a sound was made
        self.dist = None

        self.player = ac.player() #play the sound.
        self.rate = 1.0/self.speeddefault;
        self.r = rospy.Rate(1.0/self.rate);
        self.dingHappened = False;

    def config_callback(self, config, level):
        self.pitch = config['pitch']
        self.vol = config['vol']
        self.speed = config['rate']
        self.track = config['track']
        self.ding = config['ding']
        self.quantizetype = config['quantType']
        self.key = config['key']
        self.randrange = config['randRange']
        self.on = config['on']
        self.pitchdefault = config['pitchDefault']
        self.voldefault = config['volDefault']
        self.speeddefault = config['rateDefault']
        return config

    def get_dist(self, msg):
        self.dist = msg.data
        print(self.dist)
        if self.speed:
            filtereddist =  max(min(3*self.dist, 20), 2)
            self.rate = 1.0/filtereddist;
            self.r = rospy.Rate(1.0/self.rate)
        else:
            self.rate = 1.0/self.speeddefault;
            self.r = rospy.Rate(self.speeddefault)

    def run(self):
        #self.r =  #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        #self.dist = 0
        while not rospy.is_shutdown(): #Start main while loop
            #self.dist = random.random()*4
            if not self.dist is None and self.on:
                #if rospy.Time.now()-self.last_sound_time > rospy.Duration(.01) and self.walldist is not 0: #If it's been longer than 2 seconds since last sound, and the wall distance is greater than half a meter
                self.last_sound_time = rospy.Time.now() #reset sound time
                if abs(self.dist) < self.altsounddist and self.ding:
                    rewardsound=pw.Wav(self.dingsound)
                    rewardsound.play()
                    self.dingHappened = True;
                    #print('dinged')

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

                    """ MAKE THIS A PART OF THE UPDATE OF THE POSITION INSTEAD OF RECALCULATING ALL OF THIS FOR EVERY SOUND PLAYED."""
                    if (self.pitch):#self.gmode.contains('p')):
                        #print("wowowowow")
                        #midpoint = (self.maxfreqdist-self.altsounddist)/2
                        multiplier = self.maxfreq/math.pow(2, (self.maxfreqdist-self.altsounddist))
                        freq = max(min(multiplier*math.pow((abs(self.dist+random.random()*self.randrange)-self.altsounddist), 2)+65.4*2, self.maxfreq), 65.4*2)*2 #set frequency based on distance

                        #if self.dist < .7:
                        #    freq = ac.quantize(freq, quantizetype = "M", key="C")
                        #elif self.dist < 1.4:
                        #    freq = ac.quantize(freq, quantizetype = "dom7", key="G")
                        #else:
                        freq = ac.quantize(freq, quantizetype = self.quantizetype, key=self.key)
                        #print (freq):
                    else:
                        if (self.pitchdefault != ''):
                            freq = ac.freq(self.pitchdefault)
                    if (self.vol):
                        vol = (.8/(1+math.exp((self.dist)/1.5-1))+.2)*vol#insert volume function here. that goes from 1 to 0 with increasing distance to wall.
                    else:
                        vol = self.voldefault
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth

                    if self.track:
                        pass
                    else:
                        synth = ac.synth(freq, synth="sin", fadebool=False) #create synth for sound
                        synth = ac.fade_out(ac.fade_in(synth, fadeinendtime, fadeintime), fadeoutendtime, fadeouttime)
                        ac.playstream(synth, self.player, seconds = seconds, volume = vol)#synth, self.player, seconds = .5, volume = vol)
            #print(walldistself.dist)
            self.r.sleep() #wait until next iteration
            if (self.dingHappened):
                self.dingHappened = False
                rewardsound.close()


if __name__ == '__main__': #Run Code
    node = wall_audio_player()
    node.run()
