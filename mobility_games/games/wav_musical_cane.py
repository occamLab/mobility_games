#!/usr/bin/env python

import rospy
from mobility_games.auditory import play_wav as pw
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
from apriltags_ros.msg import AprilTagDetectionArray
import pyttsx
import os
from rospkg import RosPack
from dynamic_reconfigure.server import Server
from mobility_games.cfg import MusicalCaneConfig


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

        #   Set up dynamic reconfigure server
        Server(MusicalCaneConfig, self.config_callback)

        #   Set initial values for reconfigure
        self.mode = rospy.get_param("~mode", "sound")
        self.mode = 'sound'
        self.sweep_tolerance = rospy.get_param("~sweep_tolerance", "2.0")
        self.sweep_range = rospy.get_param("~sweep_range", "0.6")

        #   Set initial positions to None
        self.x = None
        self.y = None
        self.yaw = None
        self.cane_x = None
        self.cane_y = None
        self.cane_z = None

        self.start = False
        self.num_sweeps = 0

        # TODO: dynamic reconfigure
        self.count_sweeps = True
        self.reward_every_n_sweeps = 10

        #   Associate April tag ID to different wav files.
        self.tag_to_music_file = {3: "ambience2.wav", 12: "generic_music.wav"}
        self.tag_to_music_object = {}

        self.tag_to_sound_file = {3: "ding.wav", 12: "beep.wav"}
        self.tag_to_sound_object = {}

        for tag_id in self.tag_to_music_file:
            #   Create wav object of each track in music object dictionary
            self.tag_to_music_object[tag_id] = \
                pw.Wav(os.path.join(self.sound_folder,
                       self.tag_to_music_file[tag_id]))

        for tag_id in self.tag_to_sound_file:
            #   Create wav obejct of each sound file in sound disctionary
            self.tag_to_sound_object[tag_id] = \
                pw.Wav(os.path.join(self.sound_folder,
                       self.tag_to_music_file[tag_id]))

        self.reward_sound_file = None
        self.reward_sound_object = \
            pw.Wav(os.path.join(self.sound_folder, 'revving.wav'))

        self.engine = pyttsx.init()
        self.hasSpoken = False

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

    def tag_array(self, msg):
        """ Processes the april tags currently in Tango's view.
            Adds a four-integer tuple to self.tag_list for each tag detected,
            which includes x, y, and z coordinates and the tag id. """

        self.tag_list = []
        self.orientation_list = []
        self.id_list = []

        # TODO Add a wait for transform and remove try/except
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

    def config_callback(self, config, level):
        self.mode = config["mode"]
        self.sweep_tolerance = config["sweep_tolerance"]
        self.sweep_range = config["sweep_range"]
        return config

    def run_wav(self):
        """ Plays a wav file while the user is sweeping the cane consistently.
            Pauses music when the cane stops, and resumes when it continues."""

        r = rospy.Rate(10)
        print("Searching for Tango...")
        print("Searching for AR tags...")
        visible_tag = 0

        while not rospy.is_shutdown():

            if not self.start:
                print("X position: " + str(self.x),
                      "Cane x position: " + str(self.cane_x))

            #   Start program if receiving pose from Tango
            if self.x and self.cane_x and not self.start:

                #   Note initial position
                self.x_init = self.x
                self.y_init = self.y

                #   Set some initial conditions
                self.start = True
                self.cane_y_prev = self.cane_y
                last_sweep = rospy.Time.now()
                tag = 0

                #   Keep track of which tag's audio should be playing and is
                #   currently playing. -1 means no tags are applicable.
                should_play = -1
                playing = -1

                print("Connection established.")
                print("Cane found.")

                #   Set parameters for cane sweeping.
                sweep_width = self.sweep_range
                cane_points = [-sweep_width/2.0, sweep_width/2.0]
                offset_selection = 0

            if self.start:

                #   Selects the first tag in view
                if len(self.id_list):
                    if self.id_list[0] in self.tag_to_music_object:
                        visible_tag = self.id_list[0]

                #   Determine the next point the cane should reach
                offset = cane_points[offset_selection]

                #   If the cane has passed the midline (y = 0), start music
                if (self.cane_y - offset) * (self.cane_y_prev - offset) <= 0:
                    self.num_sweeps += 1
                    should_play = visible_tag
                    last_sweep = rospy.Time.now()
                    print("Reached point %s." % offset_selection)
                    offset_selection = (offset_selection + 1) % len(cane_points)

                    #   If in sound mode, play corresponding sound
                    if self.mode == "sound":
                        if self.count_sweeps:
                            self.engine.say(str(self.num_sweeps))
                            if not self.hasSpoken:
                                self.engine.runAndWait()
                                self.engine.say(str(self.num_sweeps))
                                self.engine.runAndWait()
                            self.hasSpoken = True
                        else:
                            # play sound
                            self.tag_to_sound_object[should_play].close()
                            self.tag_to_sound_object[should_play] = \
                                pw.Wav(os.path.join(self.sound_folder,
                                       self.tag_to_sound_file[should_play]))
                            self.tag_to_sound_object[should_play].play()

                        if self.num_sweeps % self.reward_every_n_sweeps == 0 \
                                and self.num_sweeps > 0:
                            self.reward_sound_object.close()
                            self.reward_sound_object = \
                                pw.Wav(os.path.join(self.sound_folder, 'revving.wav'))
                            self.reward_sound_object.play()

                # Note previous cane position
                if self.cane_y_prev != self.cane_y:
                    self.cane_y_prev = self.cane_y

                #   If no sweep in X seconds, stop music
                if rospy.Time.now() - last_sweep > rospy.Duration(self.sweep_tolerance):
                    should_play = -1

                print self.mode
                #   Music should not play if not in music mode
                if self.mode != "music":
                    should_play = -1

                #   Play music if it should be playing
                if playing != should_play and should_play >= 0:
                    self.tag_to_music_object[should_play].play()
                    playing = should_play

                #   Pause music if it should be paused
                if should_play == -1 and playing >= 0:
                    self.tag_to_music_object[playing].pause()
                    playing = -1

                #   Stop playing music to tags other than current one
                for tag in self.tag_to_music_object:
                    if self.tag_to_music_object[tag].stream.is_active() and \
                            tag != playing:
                        self.tag_to_music_object[tag].pause()

                #   Loop music if it has reached the end of the track
                if playing >= 0 and not \
                        self.tag_to_music_object[playing].stream.is_active():
                    self.tag_to_music_object[playing].close()
                    self.tag_to_music_object[playing] = \
                        pw.Wav(os.path.join(self.sound_folder,
                               self.tag_to_music_file[visible_tag]))
                    self.tag_to_music_object[playing].play()

            #   Run at 10 loops per second
            r.sleep()


if __name__ == '__main__':
    node = AudioFeedback()
    node.run_wav()
