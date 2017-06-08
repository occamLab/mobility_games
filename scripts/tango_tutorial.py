#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Header, ColorRGBA
import random
from visualization_msgs.msg import Marker
import pyttsx
from os import system

class TangoTutorialNode(object):
    def __init__(self):
        rospy.init_node('tango_tutorial')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.vis_pub = rospy.Publisher('/goal_point', Marker, queue_size=10)
        self.x = None   #   x position of Tango. Start at None because no data have been received yet.
        self.y = None
        self.yaw = None
        self.goal_distance = 1.0
        self.start = False
        self.last_play_time = None  #   Last time a beeping noise has been made
        self.last_say_time = None   #   Last time voice instructions have been made
        self.radius = 5             #   Radius, in meters, game takes place (from initial position)
        self.goal_found = None
        self.distance_to_goal = 999
        self.engine = pyttsx.init()

    def process_pose(self, msg):       #    Onngoing function that updates current x, y, and yaw
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def det_speech(self, yaw, cur_pos, goal_pos):
        #   Takes in angle, position, and position of goal. Determines relative
        #   position of goal to Tango and returns text instructions.
        dx = goal_pos[0] - cur_pos[0]
        dy = goal_pos[1] - cur_pos[1]
        angle = math.atan2(dy, dx) - yaw
        angle %= 2*math.pi      #   Makes sure angle is between 0 and 2pi
        if angle < math.pi/6:
            text = "You're on the right path!"
        elif angle < math.pi/3:
            text = "The goal is slightly to your left."
        elif angle < 2*math.pi/3:
            text = "The goal is to your left."
        elif angle < 4*math.pi/3:
            text = "The goal is behind you."
        elif angle < 5*math.pi/3:
            text = "The goal is to your right."
        elif angle < 2*math.pi:
            text = "The goal is slightly right."
        else:
            text = "Something funny happened."
        #return round(angle, 2)
        return text

    def run(self):
        r = rospy.Rate(10)  #   Runs loop at 10 times per second
        has_spoken = False
        while not rospy.is_shutdown():

            #   Because self.x starts at None, a value in self.x means Tango is
            #   receiving data.
            if self.x and not self.start:
                self.x_init = self.x
                self.y_init = self.y
                self.yaw_init = self.yaw
                self.goal_found = True
                self.start = True

            if self.goal_found:
                self.goal_rad = random.random() * self.radius       #   Generates new goal position
                self.goal_theta = random.random() * 2.0 * math.pi   #   in polar coordinates
                self.x_goal = math.cos(self.goal_theta) * self.goal_rad + self.x_init
                self.y_goal = math.sin(self.goal_theta) * self.goal_rad + self.y_init
                self.yaw_goal = self.yaw
                self.start = True
                self.goal_found = False

            if self.start and self.distance_to_goal:
                self.vis_pub.publish(Marker(header=Header(frame_id="odom", stamp=rospy.Time.now()),
                                            type=Marker.SPHERE,
                                            pose=Pose(position=Point(x=self.x_goal, y=self.y_goal)),
                                            scale=Vector3(x=1.2,y=1.2,z=1.2),
                                            color=ColorRGBA(r=1.0,a=1.0)))  #   rvis marker for goal
                self.distance_to_goal = math.sqrt((self.x - self.x_goal)**2 + (self.y - self.y_goal)**2)
                print "distance to goal", self.distance_to_goal
                if not self.last_say_time or rospy.Time.now() - self.last_say_time > rospy.Duration(10.0):
                    #   If it has been ten seconds since last speech, give voice instructions
                    self.last_say_time = rospy.Time.now()
                    speech = self.det_speech(self.yaw, (self.x, self.y), (self.x_goal, self.y_goal))
                    self.engine.say(speech)
                    if not has_spoken:
                        a = self.engine.runAndWait()
                        self.engine.say("Hello.")
                        a = self.engine.runAndWait()
                    has_spoken = True
                if ((not self.last_play_time or
                    rospy.Time.now() - self.last_play_time > rospy.Duration(6.0/(1+math.exp(-self.distance_to_goal*.3))-2.8)) and
                    (not self.last_say_time or
                    rospy.Time.now() - self.last_say_time > rospy.Duration(2.5))):

                    self.last_play_time = rospy.Time.now()
                    #system('aplay beep.wav')


            if self.start and self.distance_to_goal < 0.6:
                #   If goal is reached, make a ding and generate new goal
                system('aplay ding.wav')
                self.goal_found = True

            r.sleep()

if __name__ == '__main__':
    node = TangoTutorialNode()
    node.run()
