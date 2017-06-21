#!/usr/bin/env python

import rospy
from mobility_games.auditory import play_wav as pw
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
from apriltags_ros.msg import AprilTagDetectionArray
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
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_array)

        #   Set initial positions to None
        self.x = None
        self.y = None
        self.yaw = None
        self.cane_x = None
        self.cane_y = None
        self.cane_z = None
        self.start = False

        #   Associate April tag ID to different wav files.
        self.tag_to_music_file = {3 : "ambience2.wav", 12 : "generic_music.wav"}
        self.tag_to_music_object = {}

        for tag_id in self.tag_to_music_file:
            #   Create wav object of each track in music_dict
            self.tag_to_music_object[tag_id] = pw.Wav(os.path.join(self.sound_folder, self.tag_to_music_file[tag_id]))


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

    def tag_array(self, msg):
        """ Processes the april tags currently in Tango's view.
            Adds a four-integer tuple to self.tag_list for each tag detected,
            which includes x, y, and z coordinates and the tag id. """

        self.tag_list = []
        self.orientation_list = []
        self.id_list = []

        #TODO Add a wait for transform and remove try/except
        #   Occasionally, transform will give error because of differences in
        #   timestamp. The except provides tag ID without using tf.
        try:
            for item in msg.detections:
                newitem = self.listener.transformPose('odom', item.pose)
                x = newitem.pose.position.x
                y = newitem.pose.position.y
                z = newitem.pose.position.z
                tag_id = item.id
                self.tag_list.append((x, y, z, tag_id))
                self.tag_dict[tag_id] = (x, y, z)
                angles = euler_from_quaternion([newitem.pose.orientation.x,
                                                newitem.pose.orientation.y,
                                                newitem.pose.orientation.z,
                                                newitem.pose.orientation.w])
                self.orientation_list.append(angles)
                self.id_list.append(tag_id)
        except:
            for item in msg.detections:
                x = None
                y = None
                z = None
                tag_id = item.id
                self.tag_list.append((x, y, z, tag_id))
                self.id_list.append(tag_id)

    def run_wav(self):
        """ Plays a wav file while the user is sweeping the cane consistently.
            Pauses music when the cane stops, and resumes when it continues. """

        r = rospy.Rate(10)
        print("Searching for Tango...")
        print("Searching for AR tags...")
        visible_tag = 0

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
                should_play = -1
                playing = -1
                tag = 0

                print("Connection established.")
                print("Cane found.")

            if self.start:

                if len(self.id_list):
                    visible_tag = self.id_list[0]
                #print(". . . . .")

                #   If the cane has passed the midline (y = 0), start music
                if self.cane_y * self.cane_y_prev <= 0:
                    should_play = visible_tag
                    last_sweep = rospy.Time.now()

                #   Note previous cane position
                if self.cane_y_prev != self.cane_y:
                    self.cane_y_prev = self.cane_y

                #   If no sweep in X seconds, stop music
                if rospy.Time.now() - last_sweep > rospy.Duration(2.0):
                    should_play = -1

                #   Play music if it should be playing
                if playing != should_play and should_play >= 0:
                    self.tag_to_music_object[should_play].play()
                    playing = should_play

                #   Pause music if it should be paused
                if should_play == -1 and playing >= 0:
                    self.tag_to_music_object[playing].pause()
                    playing = -1

                for tag in self.tag_to_music_object:
                    if self.tag_to_music_object[tag].stream.is_active() and tag != playing:
                        self.tag_to_music_object[tag].pause()

                #   Loop music if it has reached the end of the track
                if playing >= 0 and not self.tag_to_music_object[playing].stream.is_active():
                    self.tag_to_music_object[playing].close()
                    self.tag_to_music_object[playing] = pw.Wav(os.path.join(self.sound_folder, self.tag_to_music_file[visible_tag]))
                    self.tag_to_music_object[playing].play()

            #   Run at 10 loops per second
            r.sleep()


if __name__ == '__main__':
    node = AudioFeedback()
    node.run_wav()
