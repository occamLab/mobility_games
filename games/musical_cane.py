#!/usr/bin/env python

import rospy
import play_wav as pw
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Header, ColorRGBA
import random
from visualization_msgs.msg import Marker
import pyttsx
from os import system
import numpy as np
import pyaudio
import wave
import time
import sys
from audio_controller import *

class AudioFeedback(object):
    def __init__(self):
        rospy.init_node('audio_feedback')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.x = None   #   x and y position of Tango
        self.y = None
        self.yaw = None
        self.start = False
        self.player = AudioIO(True)
        self.tracks = final_countdown()
        self.quarter_time = 0.25
        self.track_length = len(self.tracks[0])
        self.track_time = self.quarter_time * self.track_length
        self.music = arrays_to_sound(self.tracks, self.quarter_time)

    def process_pose(self, msg):    #   Updates position of Tango
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def run(self):
        lps = 3/self.quarter_time    #   loops per second!
        r = rospy.Rate(lps)
        beat = 1
        last_chord = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.x and not self.start:
                #   Once position signals start to be received,
                #   note initial position.
                self.x_init = self.x
                self.y_init = self.y
                self.start = True
                print("Connection established.")
            if self.x and self.start and rospy.Time.now() - last_chord >= rospy.Duration(self.quarter_time):
                dist = math.sqrt((self.x - self.x_init)**2 + (self.y - self.y_init)**2)
                #   Play variable numbers of tracks depending on distance from
                #   initial position.
                if dist < 1:
                    chord = sum(self.music[beat - 1, :])
                elif dist < 2:
                    chord = sum(self.music[beat - 1, :-1])
                elif dist < 3:
                    chord = sum(self.music[beat - 1, :-2])
                elif dist < 4:
                    chord = sum(self.music[beat - 1, :-3])
                elif dist < 5:
                    chord = sum(self.music[beat - 1, :-4])
                else:
                    chord = sum(self.music[beat - 1, :-5])
                play(chord, self.player)
                print(chord)
                beat += 1
                if beat > self.track_length:
                    beat = 1
                last_chord = rospy.Time.now()
            r.sleep()

if __name__ == '__main__':
    node = AudioFeedback()
    node.run()
