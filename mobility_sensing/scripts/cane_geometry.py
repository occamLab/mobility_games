#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from tf.transformations import quaternion_matrix
from tf.transformations import euler_from_quaternion
import numpy as np
from mobility_sensing.msg import Sweep
import math

class CaneGeometry:
    """ CaneGeometry defines a ros node that uses apriltags to define
    the geometry of a white cane.

    Parameters
    ----------
    tag_distance: float
        the distance in inches along the cane shaft, between tag and tip
    """

    def __init__(self, tag_distance):


        rospy.init_node('cane_geometry')
        rospy.Subscriber('/fisheye_undistorted/tag_detections_pose',
                         PoseArray, self.republish_cane_tip)


        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose)


        self.pub = rospy.Publisher('/sweeps', Sweep, queue_size=10)

        self.cane_x = None
        self.cane_y = None
        self.cane_z = None

        self.start = True
        self.start_dir = 0.0
        self.angle_prev = 0.0

        # self.pub = rospy.Publisher('/cane_tip', PoseStamped, queue_size=1)

        self.tag_distance = rospy.get_param('~tag_distance')

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


    def republish_cane_tip(self, tag_msg):

        # If any tags were detected
        if len(tag_msg.poses):

            # FIXME: Possibly poses[0] is not the correct one, if multiple tags
            pos = tag_msg.poses[0].position
            quat = tag_msg.poses[0].orientation

            # rotation matrix way
            # normal vector of tag is the z axis of the tag orientation
            R = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])

            in_to_m = 0.0254
            length_m = self.tag_distance * in_to_m

            # FIXME: Tip location calc is approximate.
            # Tag is not concentric with cane.

            # normal was pointing towards handle
            # opposite direction of normal is tip

            self.cane_x = pos.x - length_m * R[0,2]
            self.cane_y = pos.y - length_m * R[1,2]
            self.cane_z = pos.z - length_m * R[2,2]

            if np.isnan(self.cane_x) or np.isnan(self.cane_y) or (self.cane_x == 0 and self.cane_y == 0):
                return

            # creates a vector at the starting point
            # angle = np.arctan2(y_pos, x_pos)
            direction = np.asarray([self.cane_x, self.cane_y])

            # normalizing
            direction = direction / np.linalg.norm(direction)


            # when first starting, sets that point as the zero point by
            # keeping track of the angle from the x axis at that point
            if self.start == True:
                self.start_dir = direction
                self.start = False

            # dot product find angle between starting vector and current direction
            angle_from_starting = np.arccos(np.dot(direction, self.start_dir))


            # change in angle
            delta_angle = angle_from_starting - self.angle_prev

            delta_angle_deg = delta_angle * 57.2958


            if delta_angle_deg > 1.0 or delta_angle_deg < -1.0:
                if delta_angle < 0:

                    #tag distance correct measrument?
                    # i think tag_distance is incorrect
                    sweep_distance = length_m * math.sin(angle_from_starting / 2) * 2

                    print "CANE SWEEP!" + str(angle_from_starting * 57.2958) + " distance: " + str(sweep_distance)
                    # need to calculate distance of sweep, the angle is holding the place at the moment
                    completed_sweep = Sweep(sweep_detection="CANE SWEEP!", distance_of_sweep=sweep_distance)
                    self.pub.publish(completed_sweep)


                    self.start_dir = direction
                    self.angle_prev = 0.0
                    return

            # update the previous angle

            self.angle_prev = angle_from_starting

            # print "angle: " + str(angle_from_starting * 57.2958) +   " delta_angle: " + str(delta_angle_deg)




    def run(self):
        """ The main run loop"""
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == "__main__":
    cane = CaneGeometry(tag_distance=29.5)
    cane.run()
