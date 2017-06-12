#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose
import numpy as np
import pcl
import tf
from threading import Lock
import time
#import copy
import tf
import math

class PlaneFitter(object):
    def __init__(self):
        rospy.init_node('floor_finder')
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud)
        self.m = Lock()
        self.pub = rospy.Publisher('/plane_finder', PointCloud, queue_size=10)
        self.pub_transformed = rospy.Publisher('/cloud_transformed', PointCloud, queue_size=10)

        self.listener = tf.TransformListener()
        self.CurrP = None
        self.cloud_plane = None
        self.actualP = None
        self.planetrynum = 5
        self.verticalityThreshold = .4
        self.t = 0.0
        self.robustThreshold = 100
        self.ycutoff = 0;
        self.saved_plane_model = None;
        self.untransformed_saved_plane_model = None;
        #self.newP = False;

    def process_cloud(self, msg):
        self.m.acquire()
        self.actualP = msg
        self.actualP.points = [i for i in self.actualP.points[::10] if i.y > self.ycutoff]
        #self.CurrP = np.asarray([(p.x, p.y, p.z) for p in self.actualP.points], dtype = np.float32)
        #print "got a cloud", self.CurrP.shape
        self.m.release()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.actualP is None:
                #Setting Point Cloud
                self.m.acquire()
                #set_pose
                self.CurrP, actualPCopy = self.pcloud_transform(self.actualP, '/odom', True)

                if actualPCopy is None:
                    pass
                else:
                    #skip = False
                    cloud_filtered = pcl.PointCloud(self.CurrP) #Move To PointCloud2 Eventually!
                    #print "cloudsize: ", str(cloud_filtered.size)
                    #fil = cloud_filtered.make_passthrough_filter()
                    #fil.set_filter_field_name("z")
                    #fil.set_filter_limits(0, 999)
                    #cloud_filtered = self.UseP#fil.filter()
                    #print type(cloud_filtered), cloud_filtered, cloud_filtered[0]

                    #print "filtered data size: " + str(cloud_filtered.size)

                    #Plane
                    seg = cloud_filtered.make_segmenter()
                    seg.set_optimize_coefficients(True)
                    seg.set_model_type(pcl.SACMODEL_PLANE)
                    seg.set_method_type(pcl.SAC_RANSAC)
                    seg.set_distance_threshold(.03)
                    #seg.set
                    #self.CurrP = cloud_filtered.to_array
                    match = False
                    try:
                        for i in range(self.planetrynum):
                            #print i
                            check = False
                            usesave = None
                            indices, plane_model = seg.segment()
                            if self.saved_plane_model is not None :
                                if (abs(plane_model[0]-self.saved_plane_model[0]) < .075 and
                                    abs(plane_model[1]-self.saved_plane_model[1]) < .075 and
                                    abs(plane_model[2]-self.saved_plane_model[2]) < .075 and
                                    abs(plane_model[3]-self.saved_plane_model[3]) < .2):
                                    match = True
                                else:
                                    check = True
                            if len(indices) < self.robustThreshold:
                                #print len(indices)
                                print("No Robust Planes")

                                print "wall_distance: " + str(abs(self.untransformed_saved_plane_model[3]))
                                break
                            if abs(plane_model[2]) < self.verticalityThreshold or match:
                                #new_plane_model = None

                                self.cloud_plane = cloud_filtered.extract(indices, negative=False) #Extracting plane points
                                actualPCopy.points  = [actualPCopy.points[indx] for indx in indices]

                                untransformed_CurrP, actualPCopy = self.pcloud_transform(actualPCopy, "/depth_camera", True)
                                untransformed_plane_cloud = pcl.PointCloud(untransformed_CurrP)

                                seg = untransformed_plane_cloud.make_segmenter()
                                seg.set_optimize_coefficients(True)
                                seg.set_model_type(pcl.SACMODEL_PLANE)
                                seg.set_method_type(pcl.SAC_RANSAC)
                                seg.set_distance_threshold(.03)
                                new_indices, new_plane_model = seg.segment()

                                #if match:
                                #    self.saved_plane_model = plane_model
                                #    self.untransformed_saved_plane_model = new_plane_model
                                if check:
                                    if abs(new_plane_model[3]) > abs(self.untransformed_saved_plane_model[3]):
                                        usesave = True
                                    else:
                                        usesave = False
                                if usesave:
                                    print("SAVE USED")
                                    #print "plane model: " + str(self.untransformed_saved_plane_model)
                                    #new_plane_model = planetransform(plane_model, "/depth_camera")
                                    print "wall_distance: " + str(abs(self.untransformed_saved_plane_model[3]))
                                    break

                                #self.pub.publish(actualPCopy)

                                #print "plane model: " + str(plane_model)
                                #print "plane_size: " + str(self.cloud_plane.size)
                                print "wall_distance: " + str(abs(new_plane_model[3]))
                                self.saved_plane_model = plane_model
                                self.untransformed_saved_plane_model = new_plane_model

                                break
                            else:
                                for a in indices:
                                    actualPCopy.points[a] = 0
                                for a in indices:
                                    actualPCopy.points.remove(0)
                                cloud_filtered = pcl.PointCloud(np.asarray([(p.x, p.y, p.z) for p in actualPCopy.points], dtype = np.float32))
                                seg = cloud_filtered.make_segmenter()
                                seg.set_optimize_coefficients(True)
                                seg.set_model_type(pcl.SACMODEL_PLANE)
                                seg.set_method_type(pcl.SAC_RANSAC)
                                seg.set_distance_threshold(.03)
                                continue
                            #self.cloud_plane.to_file("cloud_plane.pcd")
                    except Exception as inst:
                        pass
                        #print inst, i
                self.actualP = None
                self.UseP = None
                self.m.release()

                #self.newP = True
            #if self.newP:
            #
            #self.newP = False
            r.sleep()
    def pcloud_transform(self, cloud, target_frame, get_points):

        try:
            if np.mean([p.z for p in cloud.points]) > 10**3:
                return None, None
            newcloud = self.listener.transformPointCloud(target_frame, cloud)
            if np.isnan(np.mean([p.z for p in newcloud.points])):
                return None, None
            self.pub_transformed.publish(newcloud)
            if get_points is True:
                points = np.asarray([(p.x, p.y, p.z) for p in newcloud.points], dtype = np.float32)
            else:
                points = None
        except Exception as inst:
            print inst
            newcloud = None
            points = None
        return points, newcloud

    def planetransform(self, plane, target_frame):
        t = self.listener.getLatestCommonTime('/odom', '/depth_camera')
        t_vals = (self.listener.lookupTransform('/odom', '/depth_camera', t))[0]
        r_quat = (self.listener.lookupTransform('/odom', '/depth_camera', t))[1]
        r_mat = tf.transformations.quaternion_matrix(r_quat)
        m = 1
        perppoint = plane[0:3] * plane[3]
        useperppoint = np.array(perppoint + [1]).T
        newperppoint = np.matmul(r_mat, useperppoint)
        distance = math.sqrt(math.pow(newperppoint[0],2) + math.pow(newperppoint[1],2) + math.pow(newperppoint[2],2))
        newperppoint = [i/distance for i in newperppoint]
        return newperppoint + [distance]


if __name__ == '__main__':
    node = PlaneFitter()
    node.run()
