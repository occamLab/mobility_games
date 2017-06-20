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
        tags_seen = []
        frame_strings = self.listener.getFrameStrings()
        for frame in frame_strings:
            # a tag of some id was found
            if 'tag_' in frame:
                tags_seen.append(frame)
        return tags_seen

    def update_AR_odom_transform(self, tag_frame):

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(tag_frame)):

            try:
                t = self.listener.getLatestCommonTime("odom", tag_frame)
                trans, rot = self.listener.lookupTransform(
                        tag_frame, "odom", t)
                print trans, rot
                self.translations[tag_frame] = trans
                self.rotations[tag_frame] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

    def broadcast_AR_odom_transform(self):

        try:
            self.broadcaster.sendTransform(
                    self.translations[self.first_tag],
                    self.rotations[self.first_tag],
                    rospy.get_rostime(),
                    "odom",
                    "AR")

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                KeyError) as e:
            print e

    def update_tag_AR_transform(self, child_tag):

        if (self.listener.frameExists(child_tag)
                and self.listener.frameExists("AR")):

            try:
                t = self.listener.getLatestCommonTime(child_tag, "AR")
                trans, rot = self.listener.lookupTransform(
                        child_tag, "AR", t)

                print trans, rot
                self.translations[child_tag] = trans
                self.rotations[child_tag] = rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

    def broadcast_tag_AR_transform(self, child_tag):

        try:
            self.broadcaster.sendTransform(
                    self.translations[child_tag],
                    self.rotations[child_tag],
                    rospy.get_rostime(),
                    "AR_" + parse_tag_id(child_tag),
                    "AR")

        except (tf.ExtrapolationException,
                tf.LookupException,
                tf.ConnectivityException,
                KeyError) as e:
            print e

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            tags_seen = self.tags_detected()

            if len(tags_seen) > 0:
                if self.first_tag is None:
                    self.first_tag = tags_seen[0]

                # update transform of first tag to odom, if possible
                self.update_AR_odom_transform(self.first_tag)

                self.broadcast_AR_odom_transform()

                # handling for first_tag completed
                tags_seen.remove(self.first_tag)

                for child_tag in tags_seen:

                    # update transform of child tag to first tag, if possible
                    self.update_tag_AR_transform(child_tag)

                    self.broadcast_tag_AR_transform(child_tag)

            r.sleep()


if __name__ == "__main__":
    node = TagFrames()
    node.run()
