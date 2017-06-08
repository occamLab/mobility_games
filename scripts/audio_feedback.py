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

class AudioFeedback(object):
    def __init__(self):
        rospy.init_node('audio_feedback')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.x = None   #   x and y position of Tango
        self.y = None
        self.yaw = None
        self.start = False
        self.mus = pw.Wav("ambience2.wav")  #   Loads music from wav file
        print("Initialization done.")

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
        lps = 10    #   loops per second!
        r = rospy.Rate(lps)
        has_spoken = False
        p = pyaudio.PyAudio #   Establish pyaudio object
        playing = False
        print("Script starting.")
        while not rospy.is_shutdown():
            if self.x and not self.start:
                self.start = True
                jump_t = 0.5    #   duration (s) jump measurements are sampled
                dist_t = 0.5    #                movement
                rot_t = 1.0     #                rotation
                distance = [0] * int(dist_t * lps)  #   Create list of keyframes
                rotation = [0] * int(rot_t * lps)   #   for movement, rotation, and
                zd = [0] * int(jump_t * lps)        #   z position.
                #   These will be used to tell whether the user is moving, jumping, or rotating.
                last_beep = rospy.Time.now()
                beep_frequency = 1
                last_whoosh = rospy.Time.now()
                whoosh_frequency = 1
                x_curr, y_curr, z_curr, yaw_curr = self.x, self.y, self.z, self.yaw

            if self.x and self.start:
                x_prev = x_curr
                y_prev = y_curr
                x_curr = self.x
                y_curr = self.y
                #   Update position for this loop

                change = math.sqrt((x_curr - x_prev)**2 + (y_curr - y_prev)**2)
                distance = [change] + distance[:-1]
                total_distance = sum(distance)  #   Finds horizontal distance since last frame
                if total_distance > 0.5 and playing == False:
                    self.mus.play() #   Play music if user has moved more than half a meter in last 500ms
                    playing = True
                elif total_distance > 0.25:
                    pass
                else:
                    #   Stop music if user has moved less than a quarter meter in last 500ms
                    self.mus.pause()
                    playing = False

                if not self.mus.stream.is_active() and playing:
                    self.mus.close()
                    self.mus = pw.Wav("ambience2.wav")
                    self.mus.play()


                z_prev = z_curr
                z_curr = self.z
                change = (z_curr - z_prev)
                zd = [change] + zd[:-1]
                total_jump = sum(zd)
                if total_jump > 0.2:
                    #   If z position has increased by at least 0.2m, user has jumped.
                    system('aplay /home/jryan/catkin_ws/src/tango_test/scripts/jomp.wav')
                    zd = [0] * int(jump_t * lps)

                yaw_prev = yaw_curr
                yaw_curr = self.yaw

                change = yaw_curr - yaw_prev
                if abs(change) > math.pi:
                    change -= 2*math.pi*np.sign(change)
                rotation = [change] + rotation[:-1]
                total_rotation = sum(rotation)

                if abs(total_rotation) > math.pi and rospy.Time.now() - last_whoosh > rospy.Duration(whoosh_frequency):
                    system('aplay /home/jryan/catkin_ws/src/tango_test/scripts/ding.wav')
                    rotation = [0] * int(rot_t * lps)
                    last_whoosh = rospy.Time.now()

                print "Distance: " + str(round(total_distance, 3))
                print "Rotation: " + str(round(total_rotation, 3))

            r.sleep()
        self.mus.close()

if __name__ == '__main__':
    node = AudioFeedback()
    node.run()
