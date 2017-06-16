#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from tf.transformations import quaternion_matrix


class ARLocalization:

    def __init__(self):
        rospy.init_node('ar_localization')
        rospy.Subscriber('/fisheye_undistorted/tag_detections_pose',
                         PoseArray, self.republish_opposing_tag)

        self.pub = rospy.Publisher('/other_tag', PoseStamped, queue_size=1)

        self.tag_distance = rospy.get_param('~tag_distance')

    def republish_opposing_tag(self, tag_msg):

        # If any tags were detected
        if len(tag_msg.poses):

            # FIXME: Possibly poses[0] is not the correct one, if multiple tags
            pos = tag_msg.poses[0].position
            quat = tag_msg.poses[0].orientation

            # rotation matrix way
            # normal vector of tag is the z axis of the tag orientation
            R = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])

            length_m = self.tag_distance

            # normal was pointing towards handle
            # opposite direction of normal is tip
            opposing_tag_position = Point(pos.x + length_m * R[0,2],
                                          pos.y + length_m * R[1,2],
                                          pos.z + length_m * R[2,2])

            opposing_tag_pose = Pose(position=opposing_tag_position,
                                     orientation=quat)

            opposing_tag_pose_stamped = PoseStamped(header=tag_msg.header,
                                                    pose=opposing_tag_pose)

            self.pub.publish(opposing_tag_pose_stamped)

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    node = ARLocalization()
    node.run()
