""" Some convenience functions for translating between various representions of a robot pose.
    TODO: nothing... you should not have to modify these """

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from copy import deepcopy

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample

def convert_translation_rotation_to_pose(translation, rotation):
    """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
    return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

def convert_pose_inverse_transform(pose):
    """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python)
    IMPORTANT NOTE! PS! PLZ READ!::::::::::::::::: THIS ONLY WORKS FOR 2D"""
    translation = np.zeros((3,1))
    translation[0] = pose.position.x
    translation[1] = pose.position.y
    translation[2] = pose.position.z
    #translation[3] = 1.0

    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    #was fine
    T = tf.transformations.quaternion_matrix(rotation)
    T[:-1, -1] = np.squeeze(np.asarray(translation))
    T = np.asmatrix(T)
    inv = T.I
    #print(inv)
    new_rot = tf.transformations.quaternion_from_matrix(inv)
    new_trans = tf.transformations.translation_from_matrix(inv)
    return (new_trans, new_rot)

def invert_transform_2(trans, rot):
    """The Sequel to the above function #Classic #Lit #Get_Wrecked_Kids"""
    T = tf.transformations.quaternion_matrix(rot)
    T[:-1, -1] = np.asarray(trans)
    T = np.asmatrix(T)
    inv = T.I
    #print(inv)
    new_rot = tf.transformations.quaternion_from_matrix(inv)
    new_trans = tf.transformations.translation_from_matrix(inv)
    return (new_trans, new_rot)

"""def convert_pose_transform(pose):
    "" Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) ""
    translation = np.zeros((4,1))
    translation[0] = pose.position.x
    translation[1] = pose.position.y
    translation[2] = pose.position.z
    translation[3] = 1.0

    rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    euler_angle = euler_from_quaternion(rotation)
    rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw
    transformed_translation = rotation.dot(translation)

    translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
    rotation = quaternion_from_matrix(rotation)
    return (translation, rotation)"""

"""def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = [quaternion0.w, quaternion0.x, quaternion0.y, quaternion0.z]
    w1, x1, y1, z1 = [quaternion0.w, quaternion0.x, quaternion0.y, quaternion0.z]
    result = np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)
    return Quaternion(result[1],result[2],result[3],result[0]);"""

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return math.atan2(math.sin(z), math.cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*math.pi - math.fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if math.fabs(d1) < math.fabs(d2):
        return d1
    else:
        return d2
