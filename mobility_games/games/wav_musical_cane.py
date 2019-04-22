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
from mobility_sensing.msg import Sweep
from gtts import gTTS
from tempfile import TemporaryFile
from pygame import mixer


class AudioFeedback(object):
    def __init__(self):
        top = RosPack().get_path('mobility_games')
        self.sound_folder = os.path.join(top, 'auditory/sound_files')
        rospy.init_node('audio_feedback')

        #   Set initial values for reconfigure
        self.mode = rospy.get_param("~mode", "sound")
        self.sweep_tolerance = rospy.get_param("~sweep_tolerance", "2.0")
        self.sweep_range = rospy.get_param("~sweep_range", "0.6")
        # self.length_of_cane = rospy.get_param("~length_of_cane", )



        # Set initial conditions
        self.start = False
        self.num_sweeps = 0

        self.playing = -1
        self.should_play = -1

        self.last_sweep = rospy.Time.now()


        self.count_sweeps = True
        self.reward_at = {}
        for num in [10, 20, 50, 100, 250, 500, 1000]:
            self.reward_at[num] = True

        self.reward_sound_file = None
        self.reward_sound_object = None

        self.counting_sound_file = None
        self.counting_sound_object = None

        self.music_track_file = None
        self.music_track_object = None

        self.engine = pyttsx.init()
        self.engine.setProperty('voice', 'english + f1')
        self.hasSpoken = False



        #   Set up dynamic reconfigure server
        Server(MusicalCaneConfig, self.config_callback)

        rospy.Subscriber('/sweeps', Sweep, self.process_sweep)



    def process_sweep(self, msg):
        """ Counts completed sweeps and speaks them aloud. When user reaches one
            of the checkpoints, plays reward sound """

        # Checks if sweep is over the minimum threshld set by the sweep range
        if msg.distance_of_sweep > self.sweep_range:
            print("FULLL COMPLETE SWEEP!!!! distance of sweep" + str(msg.distance_of_sweep))
            self.num_sweeps += 1
            self.last_sweep = rospy.Time.now()

            self.should_play = 1

            # If in sound mode, count the number of sweeps out loud
            if self.mode == "sound":
                if self.count_sweeps:

                    # number = str(self.num_sweeps)
                    # language = 'en'
                    # myobj = gTTS(text=number, lang=language, slow=False)
                    # myobj.save("current_num.mp3")
                    # os.system("mpg321 current_num.mp3")

                    self.engine.say(str(self.num_sweeps))
                    if not self.hasSpoken:
                        self.engine.runAndWait()
                        self.engine.say(str(self.num_sweeps))
                        self.engine.runAndWait()
                    self.hasSpoken = True
                else:
                    # Beep mode, beep every sweep instead of counting
                    if self.counting_sound_object:
                        self.counting_sound_object.close()
                    self.counting_sound_object = pw.Wav(self.counting_sound_file)
                    self.counting_sound_object.play()

                # play reward music when reached checkpoint
                if self.num_sweeps in self.reward_at \
                        and self.reward_at[self.num_sweeps]:
                    if self.reward_sound_object:
                        self.reward_sound_object.close()
                    self.reward_sound_object = \
                        pw.Wav(self.reward_sound_file)
                    self.reward_sound_object.play()

    def config_callback(self, config, level):
        self.mode = config["mode"]
        self.sweep_tolerance = config["sweep_tolerance"]
        self.sweep_range = config["sweep_range"]
        self.count_sweeps = config["count_sweeps"]
        # self.length_of_cane = config["length_of_cane"]

        # sound mode reward music
        self.reward_sound_file = config["rewardSound"]
        self.reward_sound_object = pw.Wav(self.reward_sound_file)

        # beep sound
        self.counting_sound_file = config["countingSound"]
        self.counting_sound_object = pw.Wav(self.counting_sound_file)

        # music mode music
        self.music_track_file = config["musicTrack"]
        self.music_track_object = pw.Wav(self.music_track_file)

        # set reward checkpoints
        for num in [10, 20, 50, 100, 250, 500, 1000]:
            self.reward_at[num] = config["reward_at_%s" % num]

        return config

    def run_wav(self):
        """ Plays a wav file while the user is sweeping the cane consistently.
            Pauses music when the cane stops, and resumes when it continues."""

        r = rospy.Rate(10)


        while not rospy.is_shutdown():

            # If no sweep in sweep tolerance time, music stops
            if rospy.Time.now() - self.last_sweep > rospy.Duration(self.sweep_tolerance):
                self.should_play = -1

            # Music should not play if not in music mode
            if self.mode != "music":
                self.should_play = -1

            # Play music if it should be playing
            if self.playing != self.should_play and self.should_play >= 0:
                self.music_track_object.play()
                self.playing = self.should_play


             # Pause music if it should be paused
            if self.should_play == -1 and self.playing >= 0:
                self.music_track_object.pause()
                self.playing = -1

            # Loop music if it has reached the end of the track
            if self.playing >= 0 and not \
                    self.music_track_object.stream.is_active():
                self.music_track_object.close()
                self.music_track_object = \
                    pw.Wav(self.music_track_file)
                self.music_track_object.play()

            r.sleep()




if __name__ == '__main__':
    node = AudioFeedback()
    node.run_wav()
