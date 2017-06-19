#!/usr/bin/env python

import rospy
import tf


class TagFrames:

    def __init__(self):
        rospy.init_node('tag_frames')
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.tags_seen = []

    def tag_detected(self):
        frame_strings = self.listener.getFrameStrings()

        for frame in frame_strings:
            # a tag of some id was found
            if 'tag_' in frame:

                # TODO: only works with one tag
                return frame

    def publish_tag_odom_transform(self, tag_frame):

        if (self.listener.frameExists("odom")
                and self.listener.frameExists(tag_frame)):

            try:
                # One way
                t = self.listener.getLatestCommonTime("odom", tag_frame)
                trans, rot = self.listener.lookupTransform("odom",
                                                           tag_frame,
                                                           t)

                print trans, rot
                self.broadcaster.sendTransform(trans,
                                               rot,
                                               rospy.get_rostime(),
                                               "odom",
                                               "AR")
            except (tf.ExtrapolationException,
                    tf.LookupException,
                    tf.ConnectivityException) as e:
                print e
                return

    def run(self):
        """ The main run loop """
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            tag_detected = self.tag_detected()
            if tag_detected:

                # if this is the first tag we saw or have seen
                if (len(self.tags_seen) == 0
                        or self.tags_seen[0] == tag_detected):
                    self.publish_tag_odom_transform(tag_detected)

            r.sleep()


if __name__ == "__main__":
    node = TagFrames()
    node.run()
