#!/usr/bin/env python

import rospy
import mobility_games.auditory.play_wav as pw
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import random
from std_msgs.msg import Header, ColorRGBA, Float64
from visualization_msgs.msg import Marker
import pyttsx
from os import system
import numpy as np 
import pyaudio
from os import system, path
import wave
from mobility_games.utils.helper_functions import angle_diff
import time 
import sys
from rospkg import RosPack
import pcl
from sensor_msgs.msg import PointCloud
from threading import Lock

class Turns(object):

    def __init__(self):
        rospy.init_node("grid_game_2")
        rospy.Subscriber("/tango_pose", PoseStamped,self.process_pose)
        rospy.Subscriber("/wall_dist", Float64, self.process_dist)
        top = RosPack().get_path('mobility_games') 
        self.sound_folder = path.join(top, 'auditory/sound_files')
        self.engine = pyttsx.init() 
        self.dingNoise = pw.Wav(path.join(self.sound_folder, "ding.wav"))
        self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
        self.beepNoise = pw.Wav(path.join(self.sound_folder, "beep3.wav"))
        self.lastDingNoise = rospy.Time.now() 
        self.lastJumpNoise = rospy.Time.now() 
        self.lastBeepNoise = rospy.Time.now() 
        self.angleList = [90,-90,180] 
        self.hasSpoken = False

    """
        This function gets the x, y, z posiition
        This function also gets the yaw which is z rotation
    """
    def process_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    """
        This function get the dist from plane_finder file
    """
    def process_dist(self,msg):
        self.dist = msg.data

    """
        This function is where you turn according z rotation
        according to the degree you have turn play onve
        you hit the goal
    """
    def turn_game(self,angle):
        self.yaw = None  
        sound = {90:"Turn Left", -90:"Turn right", 180:"Turn around"}
        speech = sound[angle]  
        self.engine.say(speech)  
        if not self.hasSpoken:   
            self.engine.runAndWait()
            self.engine.say(speech)
            self.engine.runAndWait()
        self.hasSpoken = True
        r = rospy.Rate(10) #set the loop rate
        while not self.yaw: #not reieveing data from streamer 
            print self.yaw  
            r.sleep()
            pass
        #calculate the targeted angle
        goal_angle = self.yaw + angle*math.pi/180
        goal_angle %= 2*math.pi #puts it in the form of 2pi
        threshold = 20*math.pi/180 #angle thresh hold 
        while not rospy.is_shutdown():
            #Once you are in the angle thresh hold play the sound
            if abs(angle_diff(self.yaw, goal_angle)) <= threshold: #Make sure if you hit the threshhold
                self.jumpNoise.play()
                self.lastJumpNoise = rospy.Time.now()
                break  
            r.sleep()

    """
        This function make you walk in a straight line
        play sound if you are in a straight line
        once you have reach you gaol distacne 
        it maek you turn in a random directio 
    """
    def straight(self):
        self.x = None
        self.y = None
        self.yaw = None
        self.dist = None
        r = rospy.Rate(10) #set the rate for loops
        while not self.yaw: #not recieving data from streamer
            print "NONE"
            r.sleep()
            pass
        while not self.x: #not recieving data from streamer
            print "None"
            r.sleep()
            pass
        straightAngle = self.yaw
        errorAngle = 15*math.pi/180
        pos_x = self.x #gets the x position before you start moving
        pos_y = self.y #gets the y position before you start moving
        while not rospy.is_shutdown(): #tmake sure rospy not shutdown
            #threshhold of the stragith angle 
            if abs(angle_diff(self.yaw, straightAngle)) < errorAngle:
                #play the sound if you are in a straigth line
                if rospy.Time.now() - self.lastBeepNoise > rospy.Duration(1): 
                    self.beepNoise.close()
                    self.beepNoise = pw.Wav(path.join(self.sound_folder, "beep3.wav"))
                    self.beepNoise.play()
                    self.lastBeepNoise = rospy.Time.now()
                x = self.x - pos_x
                y = self.y - pos_y
                #calculate the distance between current posiiton to the starting position
                distance = math.sqrt((x**2)+(y**2))
                print("Wall Distance: " + str(self.dist))
                #If a wall is within 1.5 meter make you turn
                if self.dist < 1.5 and self.dist > 0:
                    degree = random.choice(self.angleList)  
                    if rospy.Time.now() - self.lastJumpNoise > rospy.Duration(2):
                        self.jumpNoise.close()
                        self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
                        self.turn_game(degree)  
                    break 
                #Once you walk 1.5 meter makes you turn
                if distance >= 2:  
                    self.dingNoise.play()
                    self.lastDingNoise = rospy.Time.now()
                    degree = random.choice(self.angleList)  
                    if rospy.Time.now() - self.lastJumpNoise > rospy.Duration(2):
                        self.jumpNoise.close()
                        self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
                        self.turn_game(degree)  
                    break  
            r.sleep()

    """
        Thisfunction just run the program
    """
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if rospy.Time.now() - self.lastDingNoise > rospy.Duration(2):
                self.dingNoise.close()
                self.dingNoise = pw.Wav(path.join(node.sound_folder, "ding.wav"))
                self.straight()
            r.sleep()


"""
    The Main function 
"""
if __name__ == '__main__':
    node = Turns()
    node.run()