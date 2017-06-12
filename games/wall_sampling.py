#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl
import tf
from math import *

class WallSampling(object):
    def __init__(self):
        rospy.init_node('wall_sampling')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)
        self.listener = tf.TransformListener()  #   Starts listening

    def find_distance(self, P, index = (0, 0), printing = True):
        x_vals = np.asarray([P[:, 0] - index[0]])
        y_vals = np.asarray([P[:, 1] - index[1]])
        z_vals = np.asarray([P[:, 2]])
        newP = np.concatenate((x_vals.T, y_vals.T, z_vals.T), axis = 1)
        add_mat = np.asarray([[1], [1], [0]])
        M = np.matmul(abs(newP), add_mat) ** 2 #   List of distances from center of camera frame
        min_index = np.argmin(M)    #   Index of point closest to center of frame
        a = P[min_index, :]
        if printing:
            print "Center point xy: (%s, %s)" % (P[min_index, 0], P[min_index, 1])
            #print "Max x: %s" % min(x_vals)
            #print "Max y: %s" % max(x_vals)
            print "Distance: %s" % round(P[min_index, 2], 2)
        return a

    def calc_points(self, leg, P):
        #   Generates three points based on a fixed leg length from the camera center
        s_leg = leg * sin(pi/3)
        l_leg = leg * sin(2*pi/3)
        p1 = np.asarray(self.find_distance(P, (0, -leg), False)).T
        p2 = np.asarray(self.find_distance(P, (l_leg, s_leg), False)).T
        p3 = np.asarray(self.find_distance(P, (-l_leg, s_leg), False)).T
        return p1, p2, p3

    def calc_plane(self, points):
        #   Finds plane coefficients a, b, c, and d, where the plane is defined
        #   by the equation ax + by + cz = d (x, y, and z are unit vectors).
        p1, p2, p3 = points
        v1 = p3 - p1
        v2 = p2 - p1
        cp = np.cross(v1, v2)
        a, b, c = cp
        d = np.dot(cp, p3)
        return a, b, c, d

    def process_cloud(self, msg):
        P = np.asarray([[p.x, p.y, p.z] for p in msg.points])   #   Point cloud from depth cam

        #   Calculate four planes based on twelve points in point cloud
        points = self.calc_points(500, P)
        a1, b1, c1, d1 = self.calc_plane(points)
        points = self.calc_points(-500, P)
        a2, b2, c2, d2 = self.calc_plane(points)
        points = self.calc_points(300, P)
        a3, b3, c3, d3 = self.calc_plane(points)
        points = self.calc_points(-300, P)
        a4, b4, c4, d4 = self.calc_plane(points)

        # Normalize plane values such that c = 1
        a1, b1, c1, d1 = a1/c1, b1/c1, c1/c1, d1/c1
        a2, b2, c2, d2 = a2/c2, b2/c2, c2/c2, d2/c2
        a3, b3, c3, d3 = a3/c3, b3/c3, c3/c3, d3/c3
        a4, b4, c4, d4 = a4/c4, b4/c4, c4/c4, d4/c4

        #   Average coefficients of the four planes
        a = np.mean([a1, a2, a3, a4])
        b = np.mean([b1, b2, b3, b4])
        c = np.mean([c1, c2, c3, c4])
        d = np.mean([d1, d2, d3, d4])

        try:
            int(a)  #   Make sure 'a' isn't NaN (would return ValueError)
            if a < 10000:   #   Make sure 'a' isn't absurdly high
                print round(a, 2), round(b, 2), round(c, 2), round(d, 2)
                print "- - - - -"
        except ValueError:
            pass

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = WallSampling()
    node.run()
