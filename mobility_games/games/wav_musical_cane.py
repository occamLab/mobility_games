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

        #   Set initial positions to None
        self.x = None
        self.y = None
        self.yaw = None
        self.cane_x = None
        self.cane_y = None
        self.cane_z = None
        self.start = False

        #   Associate April tag ID to different wav files.
        self.music_dict = {0 : "ambience2.wav"}
        self.music_list = []

        for tag_id in self.music_dict:
            #   Create wav object of each track in music_dict
            self.music_list.append(pw.Wav(os.path.join(self.sound_folder, self.music_dict[tag_id])))


    def process_pose(self, msg):
        """ Updates position and orientation of Tango """

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]


    def process_cane_tip(self, msg):
        """ Updates position of the tip of the user's cane in 3D space, based
            on an AR tag 29 inches from the cane tip. """

        self.cane_x = msg.pose.position.x
        self.cane_y = msg.pose.position.y
        self.cane_z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])


    def run_wav(self):
        """ Plays a wav file while the user is sweeping the cane consistently.
            Pauses music when the cane stops, and resumes when it continues. """

        r = rospy.Rate(10)
        print("Searching for Tango...")

        while not rospy.is_shutdown():

            #   Start program if receiving pose from Tango
            if self.x and self.cane_x and not self.start:

                #   Note initial position
                self.x_init = self.x
                self.y_init = self.y

                #   Set some initial conditions
                self.start = True
                self.cane_y_prev = self.cane_y
                last_sweep = rospy.Time.now()
                should_play = False
                playing = False
                tag = 0

                print("Connection established.")

            if self.start:

                #   If the cane has passed the midline (y = 0), start music
                if self.cane_y * self.cane_y_prev <= 0:
                    should_play = True
                    last_sweep = rospy.Time.now()

                #   Note previous cane position
                if self.cane_y_prev != self.cane_y:
                    self.cane_y_prev = self.cane_y

                #   If no sweep in X seconds, stop music
                if rospy.Time.now() - last_sweep > rospy.Duration(2.0):
                    should_play = False

                #   Play music if it should be playing
                if not playing and should_play:
                    self.music_list[tag].play()
                    playing = True

                #   Pause music if it should be paused
                if not should_play and playing:
                    self.music_list[tag].pause()
                    playing = False

                #   Loop music if it has reached the end of the track
                if not self.music_list[tag].stream.is_active() and playing:
                    self.music_list[tag].close()
                    self.music_list[tag] = pw.Wav(os.path.join(self.sound_folder, self.music_dict[tag]))
                    self.music_list[tag].play()

            #   Run at 10 loops per second
            r.sleep()


if __name__ == '__main__':
    node = AudioFeedback()
    node.run_wav()
