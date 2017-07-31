#!/usr/bin/env python

import rospy
import tf
from keyboard.msg import Key
import numpy as np
from copy import deepcopy

from apriltags_ros.msg import AprilTagDetectionArray


def parse_tag_id(tag_x):
    return tag_x.split('_')[1]


class TagFrames:

    def __init__(self):
        rospy.init_node('tag_frames')
        rospy.Subscriber('/keyboard/keydown', Key, self.key_pressed)
        rospy.Subscriber('/fisheye_undistorted/tag_detections',
                        AprilTagDetectionArray,
                        self.tag_callback)
        self.AR_semantic_calibration = False
        self.AR_Find_Try = False
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.first_tag = None
        #self.current_supplement = None
        self.translations = {}
        self.rotations = {}
        self.tag_see = None

    def tags_detected(self):
        """
        Returns
        -------
        tags_seen, list of str
            the list of frame strings for the tags which
            have been seen over the history of the listener.
        """
        tags_seen = []
        frame_strings = self.listener.getFrameStrings()
        for frame in frame_strings:
            # a tag of some id was found
            if 'tag_' in frame:
                tags_seen.append(frame)
        return tags_seen

    def update_AR_odom_transform(self, tag_frame):
        """ Will cache the transform that would make AR
        the parent node and odom as the direct child.
        If a transform fails to lookup, an error is printed,
        and the previous transform remains cached """

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(tag_frame)):

            try:
                t = self.listener.getLatestCommonTime("odom", tag_frame)

                # get transform to make tag_frame (parent) & odom (child)
                trans, rot = self.listener.lookupTransform(
                        tag_frame, "odom", t)

                #print trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                pass
                #print e
##
    def broadcast_AR_odom_transform(self):
        """ Will broadcast the transform between parent node
        AR with odom as the direct child """

        try:
            self.broadcaster.sendTransform(
                    self.translations[self.first_tag],
                    self.rotations[self.first_tag],
                    rospy.get_rostime(),
                    "odom",
                    "AR")

        except (KeyError) as e:
            pass
            #print e

    def update_tag_odom_transform(self, tag_frame):
        """ Will cache the transform that would make
        odom the parent node and AR_x as the direct child.
        If tf fails to lookup the transform, the exception
        is printed, and the old transform remains cached """

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(tag_frame)):

            try:
                t = self.listener.getLatestCommonTime("odom", tag_frame)

                # get transform to make odom (parent) & tag_frame (child)
                trans, rot = self.listener.lookupTransform(
                        "odom", tag_frame, t)

                #print "UPDATING!", tag_frame, trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                pass
                #print e

    def broadcast_tag_odom_transform(self, tag_frame):
        """ Will broadcast the transform between parent node
        odom with AR_x as the direct child.  Note that this
        is a different intended TF tree structure than if
        AR_x was a direct child of AR. """
        try:
            self.broadcaster.sendTransform(
                    self.translations[tag_frame],
                    self.rotations[tag_frame],
                    rospy.get_rostime(),
                    "AR_" + parse_tag_id(tag_frame),
                    "odom")

        except (KeyError) as e:
            #pass
            print e

    def update_tag_AR_transform(self, tag_frame):
        """ Will cache the transform that would make
        AR the parent node and AR_x as the direct child.
        If tf fails to lookup the transform, the exception
        is printed, and the old transform remains cached """
        if (self.listener.frameExists("AR")
                and self.listener.frameExists(tag_frame)):

            try:
                t = self.listener.getLatestCommonTime("AR", tag_frame)

                # get transform to make AR (parent) & tag_frame (child)
                trans, rot = self.listener.lookupTransform(
                        "AR", tag_frame, t)

                #print "UPDATING!", tag_frame, trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot
                self.broadcast_tag_AR_transform(tag_frame)

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                #pass
                print e

    def supplement_update_AR_odom_transform(self, tag_frame, origin_tag):
        """ Will cache the transform that would make
        AR the parent node and AR_x as the direct child.
        If tf fails to lookup the transform, the exception
        is printed, and the old transform remains cached """
        print("AR: " + str(self.listener.frameExists("AR")))
        print("tag_frame: " + str(self.listener.frameExists(tag_frame)))
        print("AR_frame: " + str(self.listener.frameExists("AR_" + parse_tag_id(tag_frame))))
        if (self.listener.frameExists("AR")
                and self.listener.frameExists(tag_frame)
                and self.listener.frameExists("odom")
                and self.listener.frameExists("AR_" + parse_tag_id(tag_frame))):
            try:

                #convert_translation_rotation_to_pose(tag_frame.translation, tag_frame.rotation)
                t = self.listener.getLatestCommonTime("AR", tag_frame)

                # get transform to make AR (parent) & tag_frame (child)
                trans, rot = self.listener.lookupTransform(
                        tag_frame, "odom", rospy.Time(0))

                trans2, rot2 = self.listener.lookupTransform(
                        "AR", "AR_" + parse_tag_id(tag_frame), rospy.Time(0));

                #print(trans2)
                #print(rot2)
                T = tf.transformations.quaternion_matrix(rot)
                T[:-1, -1] = np.asarray(trans)

                T2 = tf.transformations.quaternion_matrix(rot2)
                T2[:-1, -1] = np.asarray(trans2)
                #print("Transformation Information:")
                #print(T.shape)
                #print(T)

                AR_to_Odom = np.matmul(T2, T)


                #print("Multiply Successful")

                #scale, shear, rot_fin, trans_fin, perspective = tf.transformations.quaternion_from_matrix decompose_matrix(AR_to_Odom)
                #rot_fin =
                """rot_fin = deepcopy(AR_to_Odom)
                rot_fin[:-1, -1] = np.zeros(3)
                trans_fin = AR_to_Odom[:-1, -1]"""

                rot_fin = tuple(tf.transformations.quaternion_from_matrix(AR_to_Odom))
                trans_fin = tuple(tf.transformations.translation_from_matrix(AR_to_Odom))
                #rot_fin[3] = 1
                #rot_fin = tuple(rot_fin)

                #print("Extracted Translation and Rotation!")

                print "FINALLY!!!: "
                #print (tag_frame)
                #print trans_fin
                #print rot_fin
                self.translations[origin_tag] = trans_fin
                self.rotations[origin_tag] = rot_fin

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

    """def TagTakesTheWheel(self, tag_frame):
        ""
        This function allows the viewed tag to take the wheel from AR and drive
        the TF tree to accuracy.
        ""
        try:
            self.broadcaster.sendTransform(
                    self.translations[tag_frame],
                    self.rotations[tag_frame],
                    rospy.get_rostime(),
                    "AR",
                    "AR_" + parse_tag_id(tag_frame))
            return True
        except (KeyError) as e:
            print e
            return False
    def TagGivesUpTheWheel(self, tag_frame):
        ""
        This function allows the viewed tag to take the wheel from AR and drive
        the TF tree to accuracy.
        ""
        try:
            self.broadcaster.sendTransform(
                    self.translations[tag_frame],
                    self.rotations[tag_frame],
                    rospy.get_rostime(),
                    "AR",
                    "AR_" + parse_tag_id(tag_frame))
            return True
        except (KeyError) as e:
            print e
            return False

    def DisregardChild(self, tag_frame):
        ""
        This function allows AR to disregard its beautiful child, odom,
        and have an odomsitter take care of odom for it.
        ""
        try:
            self.broadcaster.sendTransform(
                    self.translations[tag_frame], #placeholder translation, this broadcast isn't used for anything whatsoever
                    self.rotations[tag_frame], #placeholder rotation, this broadcast isn't used for anything whatsoever
                    rospy.get_rostime(),
                    "odom",
                    "odomsitter")

        except (KeyError) as e:
            print e"""

    def broadcast_tag_AR_transform(self, tag_frame):
        """
        Will broadcast the transform between parent nodetags_seen
        AR with AR_x as the direct child.
        """
        try:
            self.broadcaster.sendTransform(
                    self.translations[tag_frame],
                    self.rotations[tag_frame],
                    rospy.get_rostime(),
                    "AR_" + parse_tag_id(tag_frame),
                    "AR")

        except (KeyError) as e:
            #pass
            print e

    def key_pressed(self, msg):
        if msg.code == ord('y'):
            self.AR_semantic_calibration = not self.AR_semantic_calibration
        if msg.code == ord('r'):
            self.AR_Find_Try = True

    def tag_callback(self, msg):
        self.tag_see = None
        if msg.detections:
            self.tag_see = "tag_" + str(msg.detections[0].id)

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)
        tags_seen = set()
        while not rospy.is_shutdown():
            tags_seen = tags_seen.union(set(self.tags_detected()))


            if len(tags_seen) > 0:
                #print tags_seen
                if self.first_tag is None:
                    self.first_tag = list(tags_seen)[0]

                self.update_AR_odom_transform(self.first_tag)
                self.broadcast_AR_odom_transform()

                # handling for first_tag completed
                tags_seen.remove(self.first_tag)
                for child_tag in tags_seen:
                    if self.AR_semantic_calibration and self.AR_Find_Try:
                        self.update_tag_AR_transform(child_tag)

                    self.broadcast_tag_AR_transform(child_tag)
                if not self.AR_semantic_calibration and self.tag_see:
                    self.supplement_update_AR_odom_transform(self.tag_see, self.first_tag)
                    #self.current_supplement = self.tag_see
                    print("used " + str(self.tag_see) + " as a corrector.")
                    self.broadcast_AR_odom_transform()

                self.AR_Find_Try = False;

            r.sleep()


if __name__ == "__main__":
    node = TagFrames()
    node.run()
