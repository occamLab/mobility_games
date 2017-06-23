#!/usr/bin/env python

import rospy
import mobility_games.auditory.play_wav as pw
import math
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import random
from std_msgs.msg import Header, ColorRGBA
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

class Turns(object):

    def __init__(self):
        rospy.init_node("grid_game")
        rospy.Subscriber("/tango_pose", PoseStamped,self.process_pose)

        top = RosPack().get_path('mobility_games')
        self.sound_folder = path.join(top, 'auditory/sound_files')
        self.engine = pyttsx.init()
        self.dingNoise = pw.Wav(path.join(self.sound_folder, "ding.wav"))
        self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
        self.beepNoise = pw.Wav(path.join(self.sound_folder, "beep3.wav"))
        self.beep2Noise = pw.Wav(path.join(self.sound_folder, "beep2.wav"))
        self.lastDingNoise = rospy.Time.now()
        self.lastJumpNoise = rospy.Time.now()
        self.lastBeepNoise = rospy.Time.now()
        self.lastBeep2Noise = rospy.Time.now()
        self.angleList = [90,-90,180]
        self.hasSpoken = False

    def process_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]
    
    #This funcation make you turn onve reach the goal distance
    def turn_game(self,angle):
        self.yaw = None  
        sound = {90:"Turn Left", -90:"Turn right", 180:"Turn around"}
        speech = sound[angle]  
        self.engine.say(speech)  #Says the turn accordng to the angle
        if not self.hasSpoken:   #Make sures that it doens't get interrupted
            self.engine.runAndWait()
            self.engine.say(speech)
            self.engine.runAndWait()
        self.hasSpoken = True
        r = rospy.Rate(10)
        while not self.yaw:  #Let us if data is coming in
            print self.yaw  
            r.sleep()
            pass
        goal_angle = self.yaw + angle*math.pi/180
        goal_angle %= 2*math.pi
        threshold = 20*math.pi/180
        while not rospy.is_shutdown():
            if abs(angle_diff(self.yaw, goal_angle)) <= threshold: #Make sure if you hit the threshhold
                self.jumpNoise.play()
                self.lastJumpNoise = rospy.Time.now()
                break  #Break from the loop
            r.sleep()

    #This function make walk in a straight line
    def straight(self, border_x, border_y):
        self.x = None
        self.y = None
        self.yaw = None
        r = rospy.Rate(10)
        while not self.yaw:  #Check to see if data is coming in
            print "NONE"
            r.sleep()
            pass
        while not self.x:  #Check to see if data is coming in
            print "None"
            r.sleep()
            pass
        straightAngle = self.yaw
        errorAngle = 15*math.pi/180
        loc_x = self.x
        loc_y = self.y
        while not rospy.is_shutdown():
            if abs(angle_diff(self.yaw, straightAngle)) < errorAngle:  #Makes sure that you are walking in a straigt line
                if rospy.Time.now() - self.lastBeepNoise > rospy.Duration(1): #Plays the beep sound every second
                    self.beepNoise.close()
                    self.beepNoise = pw.Wav(path.join(self.sound_folder, "beep3.wav"))
                    self.beepNoise.play()
                    self.lastBeepNoise = rospy.Time.now()
                x = self.x - loc_x
                y = self.y - loc_y
                out_x = self.x - border_x
                out_y = self.y - border_y
                outside = math.sqrt((out_x**2)+(out_y**2))
                dist = math.sqrt((x**2)+(y**2))
                if(outside > 5):
                    self.outOfBound(border_x, border_y)
                    break
                if dist >= 2:  #Check to see if you have the goal of 1.5 meter
                    self.dingNoise.play()
                    self.lastDingNoise = rospy.Time.now()
                    degree = random.choice(self.angleList)  #Choose a random angle from the angleList
                    if rospy.Time.now() - self.lastJumpNoise > rospy.Duration(2):
                        self.jumpNoise.close()
                        self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
                        self.turn_game(degree)  #Enter in the turn game method
                    break
            r.sleep()

    #This function make you turn if you are out of  bounds
    def outOfBound(self, start_x, start_y):
        speak = "Out of Bounds"
        self.engine.say(speak) 
        if not self.hasSpoken: 
            self.engine.runAndWait()
            self.engine.say(speak)
            self.engine.runAndWait()
        self.hasSpoken = True
        normal = [start_x - self.x, start_y - self.y]
        theta = [0, 90, 180 , -90]
        best_angle = 0
        best_dotprod = -1
        for i in range(len(theta)):
            goal_angle = self.yaw + theta[i]*math.pi/180
            unit = [math.cos(goal_angle), math.sin(goal_angle)]
            dotprod = normal[0] * unit[0] + normal[1] * unit[1]
            if dotprod > best_dotprod:
                best_dotprod = dotprod
                best_angle = theta[i]
        if rospy.Time.now() - self.lastJumpNoise > rospy.Duration(2):
            self.jumpNoise.close()
            self.jumpNoise = pw.Wav(path.join(self.sound_folder, "jomp.wav"))
            self.turn_game(best_angle)
        x_1 = self.x
        y_1 = self.y
        dist = 0
        straightAngle = self.yaw
        errorAngle = 15*math.pi/180
        self.engine.say("Walk Straight")
        while dist < 1:
            if abs(angle_diff(self.yaw, straightAngle)) < errorAngle:  #Makes sure that you are walking in a straigt line
                if rospy.Time.now() - self.lastBeep2Noise > rospy.Duration(1): #Plays the beep sound every second
                    self.beep2Noise.close()
                    self.beep2Noise = pw.Wav(path.join(self.sound_folder, "beep2.wav"))
                    self.beep2Noise.play()
                    self.lastBeep2Noise = rospy.Time.now()
                x_2 = self.x - x_1
                y_2 = self.y - y_1
                dist = math.sqrt((x_2**2)+(y_2**2)) 



    #This function just run the program
    def run(self):
        r = rospy.Rate(10)
        self.x = None
        while not self.x:  #Check to see if data is coming in
            print "None Run Method"
            r.sleep()
            pass
        boundry_x = self.x
        boundry_y = self.y
        while not rospy.is_shutdown():
            if rospy.Time.now() - self.lastDingNoise > rospy.Duration(2):
                self.dingNoise.close()
                self.dingNoise = pw.Wav(path.join(node.sound_folder, "ding.wav"))
                self.straight(boundry_x, boundry_y)
            r.sleep()



if __name__ == '__main__':
    node = Turns()
    node.run()