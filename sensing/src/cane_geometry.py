#!/usr/bin/evn python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from tf.transformations import quaternion_matrix


class CaneGeometry:

    def __init__(self):
        rospy.init_node('cane_geometry')
        rospy.Subscriber('/fisheye_undistorted/tag_detections_pose',
                         PoseArray, self.republish_cane_tip)

        self.pub = rospy.Publisher('/cane_tip', PoseStamped, queue_size=1)

    def republish_cane_tip(self, tag_msg):

        # If any tags were detected
        if len(tag_msg.poses):

            # FIXME: Possibly poses[0] is not the correct one, if multiple tags
            pos = tag_msg.poses[0].position
            quat = tag_msg.poses[0].orientation

            # rotation matrix way
            # normal vector of tag is the z axis of the tag orientation
            R = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])

            # TODO: parameterize length between tag and cane tip
            length_in = 29.5
            in_to_m = 0.0254
            length_m = length_in * in_to_m

            # FIXME: Tip location calc is approximate.
            # Tag is not concentric with cane.

            # normal was pointing towards handle
            # opposite direction of normal is tip
            cane_tip_position = Point(pos.x - length_m * R[0,2],
                                      pos.y - length_m * R[1,2],
                                      pos.z - length_m * R[2,2])

            cane_tip_pose = Pose(position=cane_tip_position,
                                 orientation=quat)

            cane_tip_pose_stamped = PoseStamped(header=tag_msg.header,
                                                pose=cane_tip_pose)

            self.pub.publish(cane_tip_pose_stamped)

    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    cane = CaneGeometry()
    cane.run()
