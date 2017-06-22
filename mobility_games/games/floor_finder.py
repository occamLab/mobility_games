#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
import numpy as np
import pcl
import tf

class FloorFinder(object):
    def __init__(self):
        rospy.init_node('floor_finder')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)
        self.listener = tf.TransformListener()  #   Starts listening

    def find_distance(self, P, printing = True):
        x_vals = P[:, 0]
        y_vals = P[:, 1]
        z_vals = P[:, 2]
        add_mat = np.asarray([[1], [1], [0]])
        M = np.matmul(abs(P), add_mat) ** 2 #   List of distances from center of camera frame
        min_index = np.argmin(M)    #   Index of point closest to center of frame
        if printing:
            print "Center point xy: (%s, %s)" % (P[min_index, 0], P[min_index, 1])
            #print "Max x: %s" % min(x_vals)
            #print "Max y: %s" % max(x_vals)
            print "Distance: %s" % round(P[min_index, 2], 2)
        return round(P[min_index, 2], 2)

    def depth_to_odom(self, points):
        t_vals = (self.listener.lookupTransform('depth_camera', 'odom', rospy.Time.now())[0])
        r_quat = (self.listener.lookupTransform('depth_camera', 'odom', rospy.Time.now())[1])
        r_mat = tf.transformations.quaternion_matrix(r_quat)
        len_points = points.shape[0]
        ones = np.full((len_points, 1), 1)
        print points.size, ones.size
        P_cam = np.concatenate([points, ones], 1)
        P_odom = np.matmul(P_cam, r_mat)
        return P_odom[:, :3]

    def find_height(self, room_cloud, cam_cloud):
        z_vals = room_cloud[:, 2]
        floor_height = min(z_vals)
        cam_origin = self.depth_to_odom([[0, 0, 0]])
        cam_height = cam_origin[2]
        return cam_height - floor_height

    def process_cloud(self, msg):
        P = np.asarray([[p.x, p.y, p.z] for p in msg.points])   #   Point cloud from depth cam
        #print "got a cloud", P.shape, M.shape
        d = self.find_distance(P, True)
        room_cloud = self.depth_to_odom(P)
        h = self.find_height(room_cloud, P)
        print "Height: %s" % h

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = Floor()
    node.run()
