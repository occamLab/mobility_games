#!/usr/bin/env python

import rospy
import tf


def parse_tag_id(tag_x):
    return tag_x.split('_')[1]


class TagFrames:

    def __init__(self):
        rospy.init_node('tag_frames')
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.first_tag = None
        self.translations = {}
        self.rotations = {}

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
                print e

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
            print e

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

                print "UPDATING!", tag_frame, trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

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
            print e

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)
        tags_seen = set()
        while not rospy.is_shutdown():
            tags_seen = tags_seen.union(set(self.tags_detected()))
            print tags_seen

            if len(tags_seen) > 0:
                if self.first_tag is None:
                    self.first_tag = list(tags_seen)[0]

                self.update_AR_odom_transform(self.first_tag)
                self.broadcast_AR_odom_transform()

                # handling for first_tag completed
                tags_seen.remove(self.first_tag)

                for child_tag in tags_seen:

                    self.update_tag_odom_transform(child_tag)
                    self.broadcast_tag_odom_transform(child_tag)

            r.sleep()


if __name__ == "__main__":
    node = TagFrames()
    node.run()
