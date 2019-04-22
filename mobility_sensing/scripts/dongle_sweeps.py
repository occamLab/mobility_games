#!/usr/bin/env python


from dialog_iot import FoshWrapper
import time
import struct
import rospy
from geometry_msgs.msg import QuaternionStamped, Quaternion, Pose
from std_msgs.msg import Header
from mobility_sensing.msg import Sweep
import tf
from tf.transformations import quaternion_matrix
import numpy as np
import math


class DongleCaneGeometry(object):
    def __init__(self, cane_length):
        self.start = True
        self.start_dir = 0.0
        self.angle_prev = 0.0

        rospy.init_node('dongle_cane_geometry')
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('/sweeps', Sweep, queue_size=10)
        rospy.Subscriber('/dongle_orientation', QuaternionStamped, self.detect_sweeps)

        self.cane_length = rospy.get_param('~cane_length')

    def detect_sweeps(self, quaternion_msg):

        # extract orientation components from the position of the dongle
        qx = quaternion_msg.quaternion.x
        qy = quaternion_msg.quaternion.y
        qz = quaternion_msg.quaternion.z
        qw = quaternion_msg.quaternion.w

        # get transformation matrix
        matrix = quaternion_matrix([qx ,qy, qz, qw])

        # length of cane in meters
        # cane_length = 1.1684

        # unit vector multiplied by lenth of cane to get projection in the room frame
        x_pos = self.cane_length * matrix[0,2]
        y_pos = self.cane_length * matrix[1,2]


        if np.isnan(x_pos) or np.isnan(y_pos) or (x_pos == 0 and y_pos == 0):
            return


        length_on_z_axis = math.sqrt((x_pos * x_pos) + (y_pos * y_pos))

        # this threshold should probably be a cofiguration parameter and should
        # be based on the length of the cane or height of person
        if length_on_z_axis > 0.6:

            # creates a vector at the starting position
            direction = np.asarray([x_pos, y_pos])



            # normalizing
            direction = direction / np.linalg.norm(direction)


            # sets first position as direction as a reference point
            if self.start == True:
                self.start_dir = direction
                self.start = False

            # dot product find angle between starting vector and current direction
            angle_from_starting = np.arccos(np.dot(direction, self.start_dir))

            # change in angle
            delta_angle = angle_from_starting - self.angle_prev

            # change in angles from radians to degrees
            delta_angle_deg = delta_angle * 57.2958


            # setting theshold to reduce noise when there is little movement of the cane
            if delta_angle_deg > 1.0 or delta_angle_deg < -1.0:

                # when the cane changes direction
                if delta_angle < 0:

                    # distance between both edges of sweep (straight line)
                    sweep_distance = self.cane_length * math.sin(angle_from_starting / 2) * 2

                    print "CANE SWEEP!" + str(angle_from_starting * 57.2958) + " distance: " + str(sweep_distance)

                    # publishing to sweep topic
                    completed_sweep = Sweep(sweep_detection="CANE SWEEP!", distance_of_sweep=sweep_distance)
                    self.pub.publish(completed_sweep)

                    # resetting the refrence vector to the current vector and the previous angle to 0
                    self.start_dir = direction
                    self.angle_prev = 0.0
                    return

                    # save current angle
            self.angle_prev = angle_from_starting


        # print "angle: " + str(angle_from_starting * 57.2958) +   " delta_angle: " + str(delta_angle_deg)

            self.br.sendTransform((0, 0, 0),
                             (qx, qy, qz, qw),
                             rospy.Time.now(),
                             "dongle",
                             "odom")


    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = DongleCaneGeometry(cane_length=1.1684)
    node.run()
