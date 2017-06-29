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
import time

class wall_audio_player(object):
    def __init__(self):
        top = RosPack().get_path("mobility_games")
        rospy.init_node('smart_wall_audio')
        rospy.Subscriber('/smart_wall_dist', Float64, self.get_dist) #recieves wall distance
        #self.gmode = rospy.get_param('~pc_audio_gmode', 'pvr')
        self.pitch = rospy.get_param('~pitch', True) #if pitch is modulated
        self.pitchdefault = rospy.get_param('~pitchDefault', 'A4') #set pitch if not modulated
        self.vol = rospy.get_param('~vol', True) #if volume is modulated
        self.voldefault = rospy.get_param('~volDefault', .8) #set volume if not modulated
        self.speed = rospy.get_param('~rate', True) #if rate is modulated
        self.speeddefault = rospy.get_param('~rateDefault', 20); #set rate if not modulated
        self.track = rospy.get_param('~track', False) #if tracks are used
        self.ding = rospy.get_param('~ding', False) #if rewardSound is used as reward
        self.randrange = rospy.get_param('~randRange', 0); #randomness of the sound range
        self.quantizetype = rospy.get_param('~quantType', 'M') #quantize type
        self.key = rospy.get_param('~key', 'A') #quantize key
        self.on = rospy.get_param('~on', False) #pause button
        self.altsounddist = rospy.get_param('~successZoneSize', .5) #success distance
        self.rewardSound = rospy.get_param('~rewardSound', path.join(top, 'auditory/sound_files/ding.wav'))#sets path to rewardSound
        self.maxfreqdist = 5.0 #distance at which frequency is maxed out
        self.maxfreq = 2092.8 #maximum frequency
        self.minfreq = 130.8 #minimum frequency
        #self.reverse = False;
        self.last_sound_time = rospy.Time.now() #Initialize the last time a sound was made
        self.dist = None #current distance reading

        self.player = ac.player() #create audio player.
        self.rate = 1.0/self.speeddefault; #set starting rate
        self.r = rospy.Rate(1.0/self.rate); #set Rospy spinner at rate
        #self.dingHappened = False; #check if ding has happened and whether the sound should be closed.
        self.currReward = pw.Wav(self.rewardSound)
        srv = Server(WallAudioConfig, self.config_callback) #starts dynamic reconfigure server

    def config_callback(self, config, level):
        """
        Set up all dynamic reconfigure parameters.
        """
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
        self.rewardSound = config['rewardSound']
        if (not self.currReward.stream.is_active()):
            self.currReward = pw.Wav(self.rewardSound)
        return config

    def get_dist(self, msg):
        """
        Code to run when distance is updated on the smart_wall_dist node.
        """
        self.dist = msg.data #get distance
        print(self.dist) #print distance
        if self.speed: #if modulating speed
            filtereddist =  max(min(3*self.dist, 20), 2) #modulate based on this function (CHANGE THIS FUNCTION EVENTUALLY TO BE BASED ON MAXFREQDIST AND ALTSOUNDDIST)
            self.rate = 1.0/filtereddist; #create rate for sound calculations
            self.r = rospy.Rate(filtereddist) #create rospy rate for speed of sound plays.
        else:
            self.rate = 1.0/self.speeddefault; #same as above but with the defaultspeed
            self.r = rospy.Rate(self.speeddefault) #same as above but with the defaultspeed

    def run(self):
        freq = ac.freq('C5') #set default pitch to be overwritten and saved.
        vol = .8 #set default volume to be overwritten and saved.
        while not rospy.is_shutdown(): #Start main while loop
            #self.dist = random.random()*4
            if not self.dist is None and self.on: #check if is on and has a distance
                self.last_sound_time = rospy.Time.now() #reset sound time
                if abs(self.dist) < self.altsounddist and self.ding:
                    if (not self.currReward.stream.is_active()):
                        try:
                            self.currReward.close()
                        except:
                            print('failed to close old sound')
                        try:
                            self.currReward = pw.Wav(self.rewardSound)
                        except:
                            print('failed to create new sound')
                    self.currReward.play()
                    #time.sleep(1)

                else:
                    if (self.currReward.stream.is_active()):
                        self.currReward.pause()
                    freq = ac.freq('C5') #set default pitch (to be overwritten)
                    vol = .8 #set default volume (to be overwritten)
                    seconds = 5*self.rate
                    fadeinendtime = 1.0*self.rate
                    fadeoutendtime = 1.0*self.rate
                    fadeintime = .2*self.rate
                    fadeouttime = .2*self.rate

                    """ MAKE THIS A PART OF THE UPDATE OF THE POSITION INSTEAD OF RECALCULATING ALL OF THIS FOR EVERY SOUND PLAYED!!!!!"""
                    if (self.pitch): #if pitch modulation
                        multiplier = self.maxfreq/math.pow(2, (self.maxfreqdist-self.altsounddist)) #calculate multiplier based on maxfreq, maxfreqdist, and altsounddist
                        freq = max(min(multiplier*math.pow((abs(self.dist+random.random()*self.randrange)-self.altsounddist), 2)+65.4*2, self.maxfreq), 65.4*2)*2 #set frequency based on distance
                        freq = ac.quantize(freq, quantizetype = self.quantizetype, key=self.key) #quantize audio
                    else:
                        try: #try to set default pitch
                            freq = ac.freq(self.pitchdefault)
                        except: #if that doesn't work, do nothing, leaving the pitch as whatever it previously was.
                            pass
                    if (self.vol): #if volume modulation
                        vol = (.8/(1+math.exp((self.dist)/1.5-1))+.2)*vol #insert volume function here. that goes from 1 to 0 with increasing distance to wall.
                    else:
                        vol = self.voldefault
                    #self.synth = ac.delay(self.synth, 1, self.s*.2, .5) #add delay to the synth

                    if not self.track: #if no track
                        synth = ac.synth(freq, synth="sin", fadebool=False) #create synth for sound
                        synth = ac.fade_out(ac.fade_in(synth, fadeinendtime, fadeintime), fadeoutendtime, fadeouttime)
                        ac.playstream(synth, self.player, seconds = seconds, volume = vol)#synth, self.player, seconds = .5, volume = vol)
            #print(walldistself.dist)
            self.r.sleep() #wait until next iteration
            if (not self.currReward.stream.is_active()):
                self.currReward.pause()


if __name__ == '__main__': #Run Code
    node = wall_audio_player()
    node.run()
