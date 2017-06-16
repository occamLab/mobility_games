import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Vector3
from tf.transformations import euler_from_quaternion
from apriltags_ros.msg import AprilTagDetectionArray
from std_msgs.msg import Header, ColorRGBA
import math
from os import system, path
import numpy as np
import time
import sys
import tf
from mobility_games.auditory.audio_controller import *
from mobility_games.auditory.song_library import *
from copy import deepcopy
import tty
import termios
from rospkg import RosPack

class ARFind():
    def __init__(self):
        rospy.init_node('ar_find')
        rospy.Subscriber('/tango_pose',
                        PoseStamped,
                        self.process_pose)
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_array)
        self.x = None   #   x and y position of Tango
        self.y = None
        self.z = None
        self.tag_list = []
        self.first_tag = None
        self.tag_x = None
        self.tag_y = None
        self.tag_z = None
        self.yaw = None
        self.start = False
        self.listener = tf.TransformListener()  #   Starts listening
        self.soundtrack = song_sample_1()
        self.quarter_time = 0.25
        self.player = player()
        self.tag_dict = {}

    def process_pose(self, msg):    #   Updates position of Tango
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        angles = euler_from_quaternion([msg.pose.orientation.x,
                                        msg.pose.orientation.y,
                                        msg.pose.orientation.z,
                                        msg.pose.orientation.w])
        self.yaw = angles[2]

    def tag_array(self, msg):
        # Processes the april tags currently in Tango's view.
        # Adds a four-integer tuple to self.tag_list for each tag detected,
        # which includes x, y, and z coordinates and the tag id.
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

    def run(self):
        r = rospy.Rate(10)
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            if self.x and not self.start:
                self.start = True
                print("Connection established.")
                print("Searching for tags...")
            a = []
            list_of_visible_tags = [cell[3] for cell in self.tag_list]
            if len(self.tag_list) and self.start:
                if not self.first_tag and not (None in [item[1] for item in self.tag_list]):
                    self.first_tag = self.tag_list[0]
                    print("First tag found!")
                if not None in [item[1] for item in self.tag_list]:
                    for tag in self.tag_list:
                        new_tag = tag
                        dist = []
                        for i in range(0, 3):
                            dif = (new_tag[i] - self.first_tag[i])
                            #print(dif)
                            dist.append(dif**2)
                        tot_dist = math.sqrt(sum(dist))
                        a.append(tot_dist)
            if self.start and not (a == [] and list_of_visible_tags != []):
                distance_string = str([round(item, 2) for item in a])[1:-1]
                str1 = "Distances from first tag: %s" % distance_string
                list_of_visible_tags = [cell[3] for cell in self.tag_list]
                tags_string = str(list_of_visible_tags)[1:-1]
                str2 = "Tags I can see: %s" % tags_string
                print
                print(str2)
                print(str1)
                print self.tag_dict
                #print self.tag_list
            r.sleep()

    def soundscape(self):
        """ Plays different audio tracks together depending on which tags are
        currently within the frame of the camera. """

        self.music = arrays_to_sound(self.soundtrack, self.quarter_time)
        quarter_time = 0.25
        beat = 1
        last_chord = rospy.Time.now()
        r = rospy.Rate(3/quarter_time)  #   Synch ros rate so that beats line up nicely
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            if self.x and not self.start:
                self.start = True
                print("Connection established.")
                print("Searching for tags...")
            if self.start:
                list_of_visible_tags = [cell[3] for cell in self.tag_list]  #   List of tag IDs in frame
                print("Tags I can see: " + str(list_of_visible_tags))

                if rospy.Time.now() - last_chord > rospy.Duration(self.quarter_time):
                    notes = self.music[beat - 1, :]
                    chord = deepcopy(self.music[beat - 1, 0])   #   Copy so that self.music isn't changed
                    track = 1
                    tags_anticipated = range(0, len(notes))
                    for tag in tags_anticipated:    #   can be changed to a list of tag ids
                        if tag in list_of_visible_tags:
                            chord += self.music[beat - 1, track]
                        track += 1
                    play(chord, self.player)
                    beat += 1
                    if beat > len(self.music):
                        beat = 1    #   Loop sample if at end
                    last_chord = rospy.Time.now()
                    #print self.tag_list
            r.sleep()

    def plot_waypoints(self, number):
        """ Waypoint game, except waypoints are physically plotted out beforehand. """

        self.orientation_list = []
        top = RosPack().get_path('mobility_games')
        self.sound_folder = path.join(top, 'auditory/sound_files')
        waypoints = []
        r = rospy.Rate(10)
        print("Searching for Tango...")
        while not rospy.is_shutdown():
            if self.x and not self.start:
                self.start = True
                print("Connection established.")
                print("Searching for tags...")
            if len(self.tag_list) and self.start:
                if not self.first_tag and not None in [item[1] for item in self.tag_list]:
                    self.first_tag = self.tag_list[0]
                    print("First tag found!")
                    print self.first_tag
                    system('aplay ' + path.join(self.sound_folder, "ding.wav"))
            orig_settings = termios.tcgetattr(sys.stdin)
            x = 0
            if self.start and self.first_tag:
                if len(waypoints) >= number:
                    break
                print("Press enter to add a waypoint (%s/%s)...") % (len(waypoints), number)
                x=sys.stdin.read(1)
                waypoints.append((self.x, self.y, self.z))
                system('aplay ' + path.join(self.sound_folder, "beep.wav"))
                print("Waypoints:")
                for item in waypoints:
                    print(item)
                print(self.orientation_list)
            r.sleep()
        while not self.first_tag[3] in self.id_list:
            #print self.first_tag[3]
            #print self.id_list
            r.sleep()
        system('aplay ' + path.join(self.sound_folder, "ding.wav"))
        last_beep = rospy.Time.now()
        found_waypoints = 0
        for point in waypoints:
            dist = math.sqrt((self.x - point[0])**2 + (self.y - point[1])**2 + (self.z - point[2])**2)
            while dist >= 1.0:  #   Waits until user hits a waypoint
                dist = math.sqrt((self.x - point[0])**2 + (self.y - point[1])**2 + (self.z - point[2])**2)
                print(dist)
                r.sleep()
                if rospy.Time.now() - last_beep > rospy.Duration(1.0):
                    freq_to_play = max(1500 - (dist * 75), 150) #   Plays a higher pitch the closer the user is to the waypoint
                    beep = synth(freq_to_play, synth = "digitar", fade = 1.0).take(40000)
                    play(beep, self.player)
                    last_beep = rospy.Time.now()
            found_waypoints += 1
            system('aplay ' + path.join(self.sound_folder, "ding.wav"))

if __name__ == '__main__':
    node = ARFind()
    node.run()
    #node.plot_waypoints(3)
    #node.soundscape()
    #node.run()
