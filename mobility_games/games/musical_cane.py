#!/usr/bin/env python

import rospy
from mobility_games.auditory import play_wav as pw
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Header, ColorRGBA
import random
from visualization_msgs.msg import Marker
import pyttsx
import os
from os import system
import numpy as np
import pyaudio
import wave
import time
import sys
from mobility_games.auditory.audio_controller import *
from rospkg import RosPack

class AudioFeedback(object):
    def __init__(self):
        top = RosPack().get_path('mobility_games')
        self.sound_folder = os.path.join(top, 'auditory/sound_files')
        rospy.init_node('audio_feedback')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        rospy.Subscriber('/cane_tip', PoseStamped, self.process_cane_tip)
        self.x = None   #   x and y position of Tango
        self.y = None
        self.yaw = None
        self.cane_x = None
        self.cane_y = None
        self.cane_z = None
        self.start = False
        self.player = AudioIO(True)
        self.tracks = final_countdown()
        self.quarter_time = 0.125   #   Length of quarter note, in seconds.
        self.track_length = len(self.tracks[0])
        self.track_time = self.quarter_time * self.track_length
        self.music = arrays_to_sound(self.tracks, self.quarter_time, amps = [0.8, 1.0, 0.8, 1.2, 1.0, 0.6])

    def process_pose(self, msg):    #   Updates position of Tango
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def process_cane_tip(self, msg):
        #   Y is the cane's horizontal position, where y = 0 is the midline of the cane user.
        self.cane_x = msg.pose.position.x
        self.cane_y = msg.pose.position.y
        self.cane_z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def run(self):
        """ Beeps with each successful cane sweep."""

        lps = 3/self.quarter_time    #   loops per second!
        r = rospy.Rate(lps)
        beat = 1
        last_chord = rospy.Time.now()
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            print self.x
            print self.cane_y
            if self.x and self.cane_x and not self.start:
                #   Once position signals start to be received,
                #   note initial position.
                self.x_init = self.x
                self.y_init = self.y
                self.start = True
                self.cane_y_prev = self.cane_y
                print("Connection established.")
            if self.x and self.cane_x and self.start:
                if self.cane_y * self.cane_y_prev <= 0:
                    system("aplay " + os.path.join(self.sound_folder, "beep.wav"))
                    print("beep")
                self.cane_y_prev = self.cane_y
            r.sleep()

    def run_music(self):
        """ Plays music while user is successfully sweeping cane."""

        lps = 3/self.quarter_time    #   loops per second!
        r = rospy.Rate(lps)
        print lps
        beat = 1
        last_chord = rospy.Time.now()
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            print self.x
            print self.cane_y
            if self.x and self.cane_x and not self.start:
                #   Once position signals start to be received,
                #   note initial position.
                self.x_init = self.x
                self.y_init = self.y
                self.start = True
                self.cane_y_prev = self.cane_y
                playing = 0
                print("Connection established.")
                last_sweep = rospy.Time.now()
            if self.x and self.cane_x and self.start:
                if rospy.Time.now() - last_sweep > rospy.Duration(1.5):
                    playing = 0
                if self.cane_y * self.cane_y_prev <= 0:
                    #   When the product of the previous two y positions is
                    #   negative (meaning one value is positive and one is
                    #   negative), the cane has crossed the midpoiont.
                    playing = 1
                    last_sweep = rospy.Time.now()
                if self.cane_y_prev != self.cane_y:
                    self.cane_y_prev = self.cane_y
                if rospy.Time.now() - last_chord > rospy.Duration(self.quarter_time) and playing:
                    #   Play variable numbers of tracks depending on distance from
                    #   initial position.

                    chord = chord = sum(self.music[beat - 1, :])
                    play(chord, self.player)
                    print(chord)
                    beat += 1
                    if beat > self.track_length:
                        beat = 1
                    last_chord = rospy.Time.now()
            r.sleep()

if __name__ == '__main__':
    node = AudioFeedback()
    node.run_music()
