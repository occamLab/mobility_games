#!/usr/bin/env python


from dialog_iot import FoshWrapper
import time
import struct
import rospy
from geometry_msgs.msg import QuaternionStamped, Quaternion, Pose, Vector3Stamped, Vector3
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
        self.delta_angle_prev = 0.0
        self.transform = None
        

        rospy.init_node('dongle_cane_geometry')
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('/sweeps', Sweep, queue_size=10)
        # self.pub_accel = rospy.Publisher('/bumps', Bump, queue_size=10)
        rospy.Subscriber('/dongle_orientation', QuaternionStamped, self.detect_sweeps)
        rospy.Subscriber('/dongle_accel',Vector3Stamped, self.detect_bumps)

        self.cane_length = rospy.get_param('~cane_length')
        self.prev_magnitude_accel= None

    #this method uses the magnitude of acceleration to determine when the cane bumps onto an object
    #when there is a decrease in the value of the magnitude acceleration the function will print bump
        
    def detect_bumps(self, vector_msg):

        if self.transform is None:
            return
        
        ax = vector_msg.vector.x
        ay = vector_msg.vector.y
        az = vector_msg.vector.z

        
        v = np.asarray([ax, ay, az, 1])
        axprime, ayprime, azprime, _ = self.transform.dot(v)
        # if math.sqrt(axprime**2 + ayprime**2) > .5:
            # print '%.1f' % math.atan2(ayprime, axprime)
            
        magnitude_accel = math.sqrt(axprime**2 + ayprime**2 )
        #if self.prev_magnitude_accel != None:
            
            #delta_accel = abs(magnitude_accel - self.prev_magnitude_accel)

            #if magnitude_accel > 1.0:
                
                #print 'BUMP'

        self.prev_magnitude_accel = magnitude_accel

        


    def detect_sweeps(self, quaternion_msg):

        # extract orientation components from the position of the dongle
        qx = quaternion_msg.quaternion.x
        qy = quaternion_msg.quaternion.y
        qz = quaternion_msg.quaternion.z
        qw = quaternion_msg.quaternion.w

        # get transformation matrix
        matrix = quaternion_matrix([qx ,qy, qz, qw])
        self.transform = matrix
        

        # length of cane in meters
        # cane_length = 1.1684

        # unit vector multiplied by lenth of cane to get projection in the room frame
        x_pos = self.cane_length * matrix[0,2]
        y_pos = self.cane_length * matrix[1,2]
        #z_pos = self.cane_length * matrix[2,2]


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
            change_of_delta_angle = delta_angle - self.delta_angle_prev 

            #the change of the difference in the delta angle can be used to detect bumps also this can be done by setting the threshold of the delta angle and the change of the delta angle to a suitable amount in order to detect the bumps with a normal sweep 
            #ang_velocity = delta_angle/time
            if abs(delta_angle) < 0.10 and abs(change_of_delta_angle) > 0.15:
                print ('BUMP!')
                    
            #print abs(change_of_delta_angle), abs(delta_angle)
            
            #if abs(change_of_delta_angle) > 0.2:
                #print('BUMP!')

            self.delta_angle_prev = delta_angle
                

            # change in angles from radians to degrees
            delta_angle_deg = delta_angle * 57.2958


            # setting theshold to reduce noise when there is little movement of the cane
            if delta_angle_deg > 1.0 or delta_angle_deg < -1.0:

                # when the cane changes direction
                if delta_angle < 0:

                    # distance between both edges of sweep (straight line)
                    sweep_distance = self.cane_length * math.sin(angle_from_starting / 2) * 2

                    #print "CANE SWEEP!" + str(angle_from_starting * 57.2958) + " distance: " + str(sweep_distance)

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
