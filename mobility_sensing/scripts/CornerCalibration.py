#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3, PointStamped
import numpy as np
import pcl
import tf
from threading import Lock
import time
from std_msgs.msg import Header, ColorRGBA, Float64
from visualization_msgs.msg import MarkerArray, Marker
from os import system
#import copy
import tf
import math
import pickle

class CornerCalibration(object):
    def __init__(self):
        rospy.init_node('corner_calibration') #Initialize Node for Code
        rospy.Subscriber('/point_cloud', PointCloud, self.process_cloud) #Read from Tango's Point Cloud
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose) #Read from Tango's Pose
        self.pose_pub = rospy.Publisher('/corner_detector_vectors', MarkerArray, queue_size=10)

        self.m = Lock() # Create a locker for multithreading management
        self.listener = tf.TransformListener()
        self.CurrP = None #Initialize Point List
        self.actualP = None #Initialize Point_Cloud from Tango
        self.resolutionfactor = 1 #Factor by which to lower the resolution of the point cloud (i.e. if it is 10, we only look at every 10th point of the pointcloud.)
        self.verticalityThreshold = .1
        self.calibrated = False
        self.position = None;
        self.calibrationtries = 10
        self.calibrationcounter = 0
        markercolor = ColorRGBA(.7, .3, .9, .6)
        self.cornercolors = [markercolor, markercolor]
        self.directions = np.array([[1,1], [1,-1], [-1,-1], [-1,1]], dtype = np.float32)
        self.directiondenotation = ["/^","^\\","_/","\\_"]

    def process_cloud(self, msg):
        """
        This function processes the incoming point cloud from the phone an lowers the resolution of the cloud in an attempt to run a bit faster.
        This function also uses a lock to make sure that actualP isn't being used in a calculation when the code receives new data from the tango.
        """
        self.m.acquire() #Lock
        self.actualP = msg #Receive pointcloud
        self.actualP.points = [i for i in self.actualP.points[::self.resolutionfactor]] #Lower resolution by a factor of self.resolutionfactor
        self.m.release() #Unlock

    def process_pose(self, msg):
        """
        This function receives the tango position.
        It also uses a lock in order to make sure the position isn't being used in a calculation when it is updated.
        """
        self.m.acquire() #Lock
        self.position = msg.pose.position; #Get position of Tango
        self.m.release() #Unlock

    def run(self):
        time.sleep(1)
        r = rospy.Rate(3)
        cumulativecorners = np.zeros((4,3))
        while(self.calibrationcounter < self.calibrationtries and not rospy.is_shutdown()):
            self.m.acquire()# LOCK
            print "attempt"
            if not self.actualP is None and not self.position is None:
                #raw_input("Hit enter to calibrate: ")

                self.CurrP = self.getpoints(self.actualP)
                CurrP2d = self.CurrP[:, :2]
                newcorners = []
                for i in self.directions:
                    cind = self.findfurthestpoint(i, CurrP2d)
                    newcorners.append(self.CurrP[cind])
                #newnormcorners = []
                #for i in range(len(corners)):
                #    newcorner = newcorners[i]
                #    normcorners.append(newcorner/np.linalg.norm(newcorner, axis=1))
                newnormcorners = (np.array(newcorners).T/np.linalg.norm(newcorners, axis=1)).T
                self.calibrationcounter += 1
                print "step: " + str(self.calibrationcounter) + "/" + str(self.calibrationtries)
                print newnormcorners
                print
                    #print "direction: " + self.directiondenotation[i] + " || vector: "+ str(normcorners[i])
                cumulativecorners += newnormcorners

                #pickle.dump(cind0, open("corners.p", "wb" ))
                self.stamp = self.actualP.header.stamp
                self.frame_id = self.actualP.header.frame_id
            #self.actualP = None
            self.CurrP = None
            self.posiiton = None
            self.m.release()
            r.sleep()
        markers = []
        if (self.calibrationcounter >= self.calibrationtries):
            normcorners = (cumulativecorners.T/np.linalg.norm(cumulativecorners, axis=1)).T
            print "FINAL CALIBRATION RESULT"
            print normcorners
            for i in range(len(normcorners)):
                markers.append(Marker(header = Header(frame_id=self.frame_id, stamp=self.stamp),
                                      type=Marker.LINE_LIST,
                                      ns = "Corner" + str(i),
                                      scale=Vector3(x=.05,y=.05,z=.05),
                                      points = [Point(*[0,0,0]), Point(*normcorners[i])],
                                      colors = self.cornercolors))
            self.pose_pub.publish(MarkerArray(markers = markers))
            print "success"
        else:
            print "something went wrong?"
        while (not rospy.is_shutdown()):
            for i in range(len(markers)):
                markers[i].header.stamp = self.actualP.header.stamp
            self.pose_pub.publish(MarkerArray(markers = markers))
            r.sleep()

                #self.CurrP, actualPCopy = self.pcloud_transform(self.actualP, '/odom', True) #use to check if all points are in plane
    def getpoints(self, cloud):
        points = np.asarray([(p.x, p.y, p.z) for p in cloud.points], dtype = np.float32)
        return points

    def findfurthestpoint(self, direction, points):
        pind = -1
        curdot = 0
        for i in range(len(points)):
            testdot = np.dot(direction, points[i])
            if testdot > curdot:
                curdot = testdot
                pind = i
        return pind

if __name__ == '__main__': #Run Code
    node = CornerCalibration()
    node.run()
