#!/usr/bin/env python

import rospy
import tf
from keyboard.msg import Key
from geometry_msgs.msg import Pose
import numpy as np
from copy import deepcopy
try:
    import cPickle as pickle
except:
    import pickle

from apriltags_ros.msg import AprilTagDetectionArray
import os
import re
from mobility_games.utils.helper_functions import invert_transform_2, convert_translation_rotation_to_pose


def parse_tag_id(tag_x):
    return tag_x.split('_')[1]


class TagFramesSemantic:
    def __init__(self):
        #### ROS Variables ####
        rospy.init_node('tag_frames_semantic')
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_callback)
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()

        #### Editing Variables ####
        self.g2o_result_path = '/home/juicyslew/catkin_ws/result.g2o'

        #### Tracker Variables ####
        self.AR_semantic_calibration = False
        self.AR_Find_Try = False
        self.first_tag = None
        self.load_tags = set()
        self.translations = {}
        self.rotations = {}
        self.tag_see = None
        self.first = True


    def tags_detected(self):
        """
        This function provides a list of all of the frames
        for the tags that have been seen while running.

        Returns
            new_tags_seen: list of str
        """
        new_tags_seen = []                                  #initialize list
        frame_strings = self.listener.getFrameStrings()     #get names of all broadcasted tags (???)
        for frame in frame_strings:                         #for each frame
            if 'tag_' in frame:                             #if frame has name with "tag_"
                new_tags_seen.append(frame)                 #add to list
        return new_tags_seen                                #return


    def update_AR_odom_transform(self):
        """
        This function updates the AR (parent) to Odom (child) transform information.
        If it fails to find this information, the previous transform remains in effect.
        """

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(self.first_tag)):                  #check that listeners exist

            try:
                t = self.listener.getLatestCommonTime("odom", self.first_tag)    # get the latest common time
                if self.listener.canTransform(self.first_tag, "odom", t):
                    #print 'latest common time: %s' %t
                    trans, rot = self.listener.lookupTransform(                 # get updated transform
                            self.first_tag, "odom", t)
                    self.translations[self.first_tag] = trans                        # update translation and rotation
                    self.rotations[self.first_tag] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                pass
                print e
                print'0' #This error happens when the tag representing AR no longer exists.


    def broadcast_AR_odom_transform(self):
        """
        This function Broadcasts the transform from AR (parent) to odom (child).
        Using the saved translation and rotation for the origin tag
        """

        try:
            self.broadcaster.sendTransform(             # send transform
                    self.translations[self.first_tag],
                    self.rotations[self.first_tag],
                    rospy.get_rostime(),
                    "odom",
                    "AR")

        except (KeyError) as e:
            #pass
            print e
            print'-1'


    def update_tag_AR_transform(self, tag_frame):
        """
        This function updates the AR (parent) to tag (child) transform information.
        If it fails to find this information, the previous transform remains in effect.
        """
        if (self.listener.frameExists("AR")
                and self.listener.frameExists(tag_frame)):                  # check that frames exists
            try:
                t = self.listener.getLatestCommonTime("AR", tag_frame)      # get latest common time
                if self.listener.canTransform("AR", tag_frame, t):
                    trans, rot = self.listener.lookupTransform(                 # lookup translation and rotation
                            "AR", tag_frame, t)
                    self.translations[tag_frame] = trans                        # update transform.
                    self.rotations[tag_frame] = rot
                    print 'frame_name: %s' %tag_frame

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                #pass
                print e
                print('1')


    def supplement_update_AR_odom_transform(self, tag_frame, origin_tag):
        """
        This function updates the AR (parent) to odom (child) transform information BASED ON THE SUPPLEMENT TAG SEEN
        If it fails to find this information, the previous transform remains in effect.
        """
        if (self.listener.frameExists("AR")
                and self.listener.frameExists(tag_frame)
                and self.listener.frameExists("odom")
                and self.listener.frameExists("AR_" + parse_tag_id(tag_frame))):            # check that all of the required frames exist
            try:


                t = self.listener.getLatestCommonTime("AR", tag_frame)                      # get the common time between ar and the tag

                # get transform to make AR (parent) & tag_frame (child)
                trans, rot = self.listener.lookupTransform(                                 # get latest transform from tag_x to odom
                        tag_frame, "odom", rospy.Time(0))

                trans2, rot2 = self.listener.lookupTransform(                               # get latest transform from AR to AR_x
                        "AR", "AR_" + parse_tag_id(tag_frame), rospy.Time(0));

                T = tf.transformations.quaternion_matrix(rot)                               # turn transforms into 4x4 Matrices
                T[:-1, -1] = np.asarray(trans)
                T2 = tf.transformations.quaternion_matrix(rot2)
                T2[:-1, -1] = np.asarray(trans2)

                AR_to_Odom = np.matmul(T2, T)                                               # find the AR to Odom transform based on the multiplication of the other transforms

                rot_fin = tuple(tf.transformations.quaternion_from_matrix(AR_to_Odom))      # retrieve translation and rotation from transform
                trans_fin = tuple(tf.transformations.translation_from_matrix(AR_to_Odom))

                self.translations[origin_tag] = trans_fin                                   # update transform
                self.rotations[origin_tag] = rot_fin

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e
                print('2')


    def broadcast_tag_AR_transform(self, tag_frame):
        """
        This function Broadcasts the transform from AR (parent) to ar_X (child).
        Using the saved translation and rotation for the origin tag
        """
        try:
            self.broadcaster.sendTransform( # Send AR to AR_X transform
                    self.translations[tag_frame],
                    self.rotations[tag_frame],
                    rospy.get_rostime(),
                    "AR_" + parse_tag_id(tag_frame),
                    "AR")

        except (KeyError) as e:
            pass
            #print "Key Error" #THIS ERROR HAPPENS WHEN TAG DOESN'T EXIST IN LIST YET. If the person has not yet recorded a tag but the phone has scene it.


    def ReadG2O(self, path):
        """
        This function reads through the G2O file and returns all of the vertex and edge information.
        """
        result = None
        with open(path, 'rb') as g2o_result:                    #open g2o path
            result = g2o_result.read()                          #read out values
        regexpr = "VERTEX_SE3:QUAT ([0-9]|[0-9][0-9]|[1-4][0-9][0-9]|5[0-7][0-9]|58[0-6])( .*)" # Set Vertex finding pattern for vertices of 0 to 586
        fixexpr = "FIX ([0-9]|[0-9][0-9]|[0-9][0-9][0-9])\n"                                    # Set fix finding pattern for any 3 digit number (it will only ever be a tag so it doesn't need to specify between 0 to 586)
        pattern = re.compile(regexpr)                               # create patterns
        fixpattern = re.compile(fixexpr)
        allinfo = re.findall(pattern, result)                       # find all instances of each pattern in the file
        firstinfo = re.findall(fixpattern, result)
        self.first_tag = "tag_%s" % str(firstinfo[0])               # set first_tag to the fixed vertex id
        print("first tag: %s" % str(self.first_tag))                # print out first tag

        for line in allinfo: #iterate through AR vertices
            tag_id = "tag_%s" % str(line[0])                        # set tag_id string
            line = line[1].strip()                                  # strip whitespaces from transform information
            #print(line)
            taginfo = line.split(' ')                               # split based on spaces into a list
            taginfo = [float(item) for item in taginfo]             # cast all values to floats

            self.translations[tag_id] = tuple(taginfo[0:3])         # update transform based on g2o result
            self.rotations[tag_id] = tuple(taginfo[3:])


    def key_pressed(self, msg):
        """
        This is the callback function for the ROS keyboard
        """
        if msg.code == ord('y'):
            """
            Toggle AR Calibration
            """
            self.AR_semantic_calibration = not self.AR_semantic_calibration
        if msg.code == ord('r'):
            """
            Try to find a tag.
            """
            self.AR_Find_Try = True
        if msg.code == ord('s'):
            """
            Save AR Transforms.
            """
            print("AR TRANSFORMS SAVED")
            with open('/home/juicyslew/catkin_ws/saved_AR.pkl', 'wb') as f:
                pickle.dump((self.translations, self.rotations, self.first_tag), f)     #dump information into file f with pickle
        if msg.code == ord('l'):
            """
            Load AR Transforms.
            """
            print("AR TRANSFORMS LOADED")
            container = None
            g2o = True
            if os.path.exists(self.g2o_result_path) and os.path.getsize(self.g2o_result_path) > 0: #if there is a g2o file and it isn't empty
                self.ReadG2O(self.g2o_result_path) #if there exists a g2o path to read, use it and update transforms based on that
            else: #if there isn't a g2o file
                with open('/home/juicyslew/catkin_ws/saved_AR.pkl', 'rb') as f:     #read from AR file saved
                    container = pickle.load(f)          # load up
                    self.translations = container[0]    # split loaded pickle into translations...
                    self.rotations = container[1]       # ...rotations...
                    self.first_tag = container[2]       # and the origin tag id
                    g2o = False                         # save the fact that g2o is not happening.
            self.load_tags= self.load_tags.union(set(self.translations.keys()))     # update self.load_tags with the loaded ar tags
            if len(self.load_tags) > 0:                                             # If there are any tags loaded
                self.load_tags.remove(self.first_tag)                               # Remove the first tag from the list

                #pose = convert_translation_rotation_to_pose(self.translations[self.first_tag], self.rotations[self.first_tag])
                (trans, rot) = invert_transform_2(self.translations[self.first_tag], self.rotations[self.first_tag]) #get inverted translation and rotation

                self.broadcast_AR_odom_transform()                                      # broadcast the AR to odom transform so that AR exists as we prepare to broadcast the rest of the tags.

                AR_T_inv = tf.transformations.quaternion_matrix(rot)                    # create the 4x4 matrix for the
                AR_T_inv[:-1, -1] = np.squeeze(np.asarray(trans))

                for child_tag in self.load_tags:                                        # iterate through the loaded tags
                    T = tf.transformations.quaternion_matrix(self.rotations[child_tag]) # create 4x4 transformation matrix for current tag
                    T[:-1, -1] = np.asarray(self.translations[child_tag])

                    tag_to_AR = np.matmul(AR_T_inv, T)                                  # Matrix multiply the transformation matrices to achieve the transform from tag to AR

                    self.rotations[child_tag] = tuple(tf.transformations.quaternion_from_matrix(tag_to_AR))     #update transform
                    self.translations[child_tag] = tuple(tf.transformations.translation_from_matrix(tag_to_AR))

                    self.broadcast_tag_AR_transform(child_tag) #broadcast the tag to AR transform


    def tag_callback(self, msg):
        """
        This is the callback function for the April Tag Detections.
        (It is run when tags are detected.)
        """
        self.tag_see = None
        if msg.detections:
            self.tag_see = "tag_" + str(msg.detections[0].id)


    def run(self):
        """
        The main run loop
        """
        r = rospy.Rate(10)                                                  # attempt to run 10 times a second
        tags_seen = set()                                                   # initialize tags_seen
        while not rospy.is_shutdown():                                      # start while loop
            tags_seen = tags_seen.union(self.load_tags)                     # add loaded tags to tags_seen (if they aren't there yet)
            tags_seen = tags_seen.union(set(self.tags_detected()))          # add detected tags to tags_seen (if they aren't there yet)
            if len(tags_seen) > 0 and ((self.AR_Find_Try and self.AR_semantic_calibration) or self.first_tag != None):                                          # if there are tags that have been seen.
                if self.first_tag is None:                                  # if the first tag hasn't been set
                    self.first_tag = list(tags_seen)[0]                     # set the first tag to the first in the list of seen tags
                if self.first_tag in tags_seen and (not self.AR_semantic_calibration or self.first):
                    self.first = False
                    self.update_AR_odom_transform()                         # update the transformation from AR to odom with the first tag
                self.broadcast_AR_odom_transform()                          # broadcast the AR to odomm transform
                if self.first_tag in tags_seen:                             # if first tag is still in the seen tags,
                    tags_seen.remove(self.first_tag)                        # remove it.
                for child_tag in tags_seen:                                 # iterate through the seen tags (supplemental)
                    if self.AR_semantic_calibration and self.AR_Find_Try:   # if in AR semantic calibration and tried to find AR tags
                        self.update_tag_AR_transform(child_tag)             # update the tag to AR transform.
                    self.broadcast_tag_AR_transform(child_tag)              # broadcast the tag to AR transform.

                if not self.AR_semantic_calibration and self.tag_see:       # if AR calibration is on and we have seen a tag,
                    self.supplement_update_AR_odom_transform(self.tag_see, self.first_tag)  #update the AR to odom transform based on the seen tag.
                    print("used " + str(self.tag_see) + " as a corrector.")
                    self.broadcast_AR_odom_transform()                      # broadcast the AR to odom transform that was just updated.
                self.AR_Find_Try = False                                    # turn of the attempt to find AR
            r.sleep()



if __name__ == "__main__":
    node = TagFramesSemantic()
    node.run()
