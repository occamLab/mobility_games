#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
import math
from std_msgs.msg import Header, ColorRGBA
import random
from visualization_msgs.msg import Marker
import pyttsx
from os import system, path
from rospkg import RosPack
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header, ColorRGBA
from keyboard.msg import Key


class Calibration(object):
    def __init__(self):
        self.calibration_mode = True
        self.tag_id_to_name = {}
        self.tag_name_to_id = {}
        self.visited_tags = []

        top = RosPack().get_path('mobility_games')
        rospy.init_node('semantic_waypoints')
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)
        self.x = None   #   x position of Tango. Start at None because no data have been received yet.
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_callback)
        rospy.Subscriber('/keyboard/keydown', Key, self.run_mode)
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

    def tag_callback(self, msg):
        # Processes the april tags currently in Tango's view.
        # Ask user to name april tags
        if self.calibration_mode:
            if msg.detections:
                tag_id = msg.detections[0].id
                
                # Only prompt user to input tag name once
                if not tag_id in self.visited_tags:
                    self.visited_tags.append(tag_id); 
                    tag_name = raw_input("Name the AR Tag: "); 
                    self.tag_id_to_name[tag_id] = tag_name
                    self.tag_name_to_id[tag_name] = tag_id
                    print self.tag_id_to_name
        
        # Idnetify april tags with given string names            
        else:
            if msg.detections:
                tag_id = msg.detections[0].id
                if not tag_id in self.visited_tags: 
                    self.visited_tags.append(tag_id)
                    print ("Found " + self.tag_id_to_name[tag_id])

    def run_mode(self, msg):
        # Switch to run mode
        if msg.code == 97:
            print "Starting Run Mode..."
            self.calibration_mode = False
            self.visited_tags = []
            print self.tag_id_to_name

    def run(self):
        r = rospy.Rate(10)  #   Runs loop at 10 times per second
        has_spoken = False
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    calibration_node = Calibration()
    calibration_node.run()
