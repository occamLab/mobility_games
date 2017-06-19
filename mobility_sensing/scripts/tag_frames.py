#!/usr/bin/env python

import rospy
import tf


class TagFrames:

    def __init__(self):
        rospy.init_node('tag_frames')
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.first_tag = None

    def tags_detected(self):
        tags_seen = []
        frame_strings = self.listener.getFrameStrings()
        for frame in frame_strings:
            # a tag of some id was found
            if 'tag_' in frame:
                tags_seen.append(frame)
        return tags_seen

    def update_tag_odom_transform(self, tag_frame):

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(tag_frame)):

            try:
                t = self.listener.getLatestCommonTime("odom", tag_frame)
                # self.AR_trans, self.AR_rot = self.listener.lookupTransform(
                #         "odom", tag_frame, t)
                self.AR_trans, self.AR_rot = self.listener.lookupTransform(
                        tag_frame, "odom", t)
                print self.AR_trans, self.AR_rot
                print self.AR_trans, self.AR_rot

            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            tags_seen = self.tags_detected()

            if len(tags_seen) > 0:
                if self.first_tag is None:
                    self.first_tag = tags_seen[0]

                # update AR_trans and AR_rot, if possible
                self.update_tag_odom_transform(self.first_tag)

                self.broadcaster.sendTransform(self.AR_trans,
                                               self.AR_rot,
                                               rospy.get_rostime(),
                                               "odom",
                                               "AR")

            r.sleep()


if __name__ == "__main__":
    node = TagFrames()
    node.run()
