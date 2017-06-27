#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3Stamped, Vector3
import numpy as np
import pcl
import tf
from threading import Lock
import time
from std_msgs.msg import Header, ColorRGBA, Float64
from visualization_msgs.msg import Marker, MarkerArray
from os import system
#import copy
import tf
import math
import mobility_games.auditory.audio_controller as ac
from audiolazy import *
import sys
from dynamic_reconfigure.server import Server
from mobility_sensing.cfg import ClosestWallConfig

class ClosestWallFinder(object):
    def __init__(self, gameplay, visualized):#, corner_visualized):
        self.gameplay = gameplay
        self.visualized = visualized
        rospy.init_node('close_wall_finder') #Initialize Node for Code
        rospy.Subscriber('/point_cloud', PointCloud2, self.process_cloud) #Read from Tango's Point Cloud
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose) #Read from Tango's Pose
        srv = Server(ClosestWallConfig, self.config_callback)

        self.m = Lock() # Create a locker for multithreading management
        self.pub = rospy.Publisher('/smart_wall_finder', PointCloud2, queue_size=10) #Be ready to publish pointclouds to plane_finder, cloud_transformed, and plane_lines
        self.vis_pub = rospy.Publisher('/smart_wall_viz', Marker, queue_size=10)
        self.dist_pub = rospy.Publisher('/smart_wall_dist', Float64, queue_size=10)

        self.listener = tf.TransformListener() #Initialize Transformer class that allows for easy coordinate frame transformations
        self.CurrP = None #Initialize Point List
        self.actualP = None #Initialize Point_Cloud from Tango
        self.resolutionfactor = rospy.get_param('~pc_Resolution', 5) #Factor by which to lower the resolution of the point cloud (i.e. if it is 10, we only look at every 10th point of the pointcloud.)
        self.planetrynum = 5 #Set Number of Times to try to find a plane before giving up
        self.abc_match_threshold = .075 #amount of difference there can be between a, b, and c parameters in plane_models without assuming the current wall is different from the previous wall. !!!(unsure about what this number should be)
        self.d_match_threshold = .2 #amount of difference there can be between d parameter in plane_models without assuming the current wall is different from the previous wall. !!!(unsure about what this number should be)
        self.verticalityThreshold = rospy.get_param('~pc_vertThreshold', .2) #Set how vertical walls need to be (the maximum z component of the normal vector of the plane.) !!!(unsure about what this number should be)
        self.t = 0.0 #Set current Rostime
        self.robustThreshold = rospy.get_param('~pc_RobustThreshold', 300) #Minimum number of points required to find plane
        self.ycutoff = 5; #How much of the phone's view is used (0 uses only the right side of the phone camera, a positive number will allow for more on the left to be seen as well, and a negative number will favor only the most right points)
        self.saved_plane_model = None; #Initialize saved plane
        self.planeDistThreshold = rospy.get_param('~pc_planeDistThreshold', .03);

        self.position = None; #Initialize current phone location
        self.walldist = 0; #Initialize Distance to Wall
        self.minimum_non_corner_dist = .5 #minimum distance a corner has to be from the screen edge to be considered an actual corner

        self.linelength = 10 #Determines length of drawn line in rviz
        self.linepoints = None #Initialize line points for wall drawing in rviz
        linecolor = ColorRGBA(.25,.42,.88,1) #Initialize color of the wall drawing in rviz
        self.linecolors = [linecolor, linecolor] #Initialize gradient of color of wall drawing in rviz

    def config_callback(self, config, level):
        self.robustThreshold = config["pc_RobustThreshold"]
        self.resolutionfactor = config["pc_Resolution"]
        self.verticalityThreshold = config["pc_vertThreshold"]
        self.planeDistThreshold = config["pc_planeDistThreshold"]
        return config

    def on_new_point_cloud(data):
        pc = pc2.read_points(data, skip_nans=True, field_names=("x", "y", "z"))
        pc_list = []
        for p in pc:
            pc_list.append( [p[0],p[1],p[2]] )

    """def process_cloud(self, msg):
        ""
        This function processes the incoming point cloud from the phone an lowers the resolution of the cloud in an attempt to run a bit faster.
        This function also uses a lock to make sure that actualP isn't being used in a calculation when the code receives new data from the tango.
        ""
        self.m.acquire() #Lock
        self.actualP = msg #Receive pointcloud
        #self.actualP.points = [i for i in self.actualP.points[::self.resolutionfactor] if i.y < self.ycutoff] #Lower resolution by a factor of self.resolutionfactor
        self.m.release() #Unlock"""

    def process_pose(self, msg):
        """
        This function receives the tango position.
        It also uses a lock in order to make sure the position isn't being used in a calculation when it is updated.
        """
        self.m.acquire() #Lock
        self.position = msg.pose.position; #Get position of Tango
        self.m.release() #Unlock

    def plane_distance(self, saved_plane_model, position):
        """
        This function uses a formulat to find the distance to a plane_model in the odom reference frame to the phone's position (also in the odom reference frame)
        the equation used is abs(a*x + b*y + c*z + d) / sqrt(a^2 + b^2 + c^2) where a, b, c, and d are the plane_model in the form (ax+by+cz+d = 0) and x, y, and z are the position of the phone
        """
        newdist = abs(position.x*saved_plane_model[0] + position.y*saved_plane_model[1] + position.z*saved_plane_model[2] + saved_plane_model[3])/math.sqrt(math.pow(saved_plane_model[0],2) + math.pow(saved_plane_model[1],2) + math.pow(saved_plane_model[2],2))
        return newdist

    def run(self):
        """
        This function holds the main loop that continuously finds walls and updates the wall distance
        """
        r = rospy.Rate(3) #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        while not rospy.is_shutdown(): #Start main while loop
            if not self.actualP is None and not self.position is None: #and not np.any(self.camera_corners) is None: #Wait until phone has found a pointcloud and a phone position
                self.m.acquire()# LOCK
                self.CurrP, actualPCopy = self.pcloud_transform(self.actualP, '/odom', True)
                if actualPCopy is None: #If something went wrong with the transform (if the transforms haven't been set yet or some timing issues happen)
                    pass #continue loop
                else: #otherwise:
                    cloud_filtered = pcl.PointCloud(actualPCopy) #Convert points into pcl Point Cloud (different from Ros point cloud)
                    seg = cloud_filtered.make_segmenter() #Set segmenter (tries to find segments in pointcloud)
                    seg.set_optimize_coefficients(True) #Not sure what this is
                    seg.set_model_type(pcl.SACMODEL_PLANE) #Set model that segmenter finds to a plane
                    seg.set_method_type(pcl.SAC_RANSAC) #Set method for finding plane to RANSAC algorithm
                    seg.set_distance_threshold(self.planeDistThreshold) #Not sure what this is
                    match = False #The variable that determines if the previous plane is the same plane as found now.
                    try: #Start Try statment (a lot can go wrong in the following code from timing issues, to not having enough data to set a plane.  In the end, the try except is just a much easier and probably quicker way to skip the current iteration and move to the next one)
                        for i in range(self.planetrynum): #Continue trying to find planes for self.planetrynum times.
                            if len(actualPCopy.points) < 5:
                                break
                            check = False #variable that determines if code should check if the current wall or previous wall is closer.
                            usesave = None #variable that determines whether or not to use the current wall or the previous wall.
                            indices, plane_model = seg.segment() #Receive plane_model [a, b, c, d] for equation (ax + by + cz + d = 0), also receive indices in the plane
                            if self.saved_plane_model is not None : #If there is a previous plane
                                if (abs(plane_model[0]-self.saved_plane_model[0]) < self.abc_match_threshold and
                                    abs(plane_model[1]-self.saved_plane_model[1]) < self.abc_match_threshold and
                                    abs(plane_model[2]-self.saved_plane_model[2]) < self.abc_match_threshold and
                                    abs(plane_model[3]-self.saved_plane_model[3]) < self.d_match_threshold): #See if current plane is relatively close to previous plane
                                    match = True #The walls match
                                else:
                                    check = True #The walls don't match and we should find which is closer.
                            if len(indices) < self.robustThreshold: #If the wall isn't robust enough
                                print("No Robust Planes") #Print that the wall is bad
                                if self.saved_plane_model: #if there is a saved_plane_model
                                    newdist = self.plane_distance(self.saved_plane_model, self.position) #Use saved_plane_model
                                    self.linepoints = self.gettangentpoints(self.saved_plane_model) #Find points for line to draw in rviz
                                    self.walldist = newdist #Set walldistance
                                break #skip to next iteration of while loop
                            if abs(plane_model[2]) < self.verticalityThreshold or match: #If the plane is vertical enough or the plane matches !!!(the "or match" may allow for finding a vertical wall that slopes into something horizontal.)
                                new_plane_dist= self.plane_distance(plane_model, self.position) #Find distance to newly found plane
                                if check: #if the current plane and previous plane were different (i.e. if check)
                                    if abs(new_plane_dist) > self.walldist: #If the old plane was closer
                                        plist = []
                                        for ind in indices:
                                            p = actualPCopy.points[ind]
                                            plist.append((p.x, p.y, p.z))
                                        plane_points = np.array(plist, dtype = np.float32)
                                        if self.PassedPlane(self.saved_plane_model, plane_points, self.position):
                                            usesave = False
                                        else:
                                            usesave = True #Use the save
                                    else: #otherwise
                                        usesave = False #Use the current plane
                                if usesave: #If save is used
                                    print("SAVE USED") #Print that save is used
                                    #if self.saved_plane_model: #If there is a saved plane
                                    newdist = self.plane_distance(self.saved_plane_model, self.position) #set distance

                                    #else:
                                    #    newdist = None
                                    self.linepoints = self.gettangentpoints(self.saved_plane_model) #make line for rviz
                                    self.walldist = newdist # set walldist
                                    break #skip to next iteration of while loop

                                self.walldist = new_plane_dist #Set walldist to new plane distance
                                actualPCopy.points  = [actualPCopy.points[indx] for indx in indices] #if np.linalg.norm(self.posdiff([actualPCopy.points[indx].x, actualPCopy.points[indx].y, actualPCopy.points[indx].z])) < math.sqrt(math.pow(self.walldist,2) + math.pow(self.maxcornerdist,2))] #Set ROS pointcloud to only include plane points
                                numericpoints = np.array([(p.x, p.y, p.z) for p in actualPCopy.points], dtype = np.float32)


                                self.linepoints = self.gettangentpoints(plane_model) #Make line for RVIZ

                                self.saved_plane_model = plane_model #set saved plane to the current plane
                                break #start next wall find
                            else: #if the plane didn't make the cut, and wasn't a wall
                                for a in indices: #loop through all the indices in the plane found
                                    actualPCopy.points[a] = 0 #set those indices to 0
                                for a in indices: #loop through all the indices again
                                    actualPCopy.points.remove(0) #remove the first 0 found (which all in all, removes all of the points in the plane)
                                if len(actualPCopy.points) < 5:
                                    break
                                cloud_filtered = pcl.PointCloud(np.asarray([(p.x, p.y, p.z) for p in actualPCopy.points], dtype = np.float32)) # set new pcl pointcloud that uses the updated ROS pointcloud (which doesn't have the indices in the nonwall plane that was found)

                                #MAKE NEW SEGMENTER TO BE READY TO FIND THE PLANE
                                seg = cloud_filtered.make_segmenter()
                                seg.set_optimize_coefficients(True)
                                seg.set_model_type(pcl.SACMODEL_PLANE)
                                seg.set_method_type(pcl.SAC_RANSAC)
                                seg.set_distance_threshold(self.planeDistThreshold)
                                continue #try to find another plane, ignore the one previously found
                    except Exception as inst: #If something goes wrong
                        pass #ignore it
                        #print inst #print it
                    print "wall_distance: " + str(abs(self.walldist)) #print the current wall distance
                    self.pub.publish(actualPCopy)
                    self.dist_pub.publish(self.walldist)

                    if (self.visualized and self.linepoints):
                        self.vis_pub.publish(Marker(header=Header(frame_id="odom", stamp=self.actualP.header.stamp),
                                                    type=Marker.LINE_LIST,
                                                    ns = "current_plane",
                                                    scale=Vector3(x=.1,y=.1,z=.1),
                                                    points = self.linepoints,
                                                    colors = self.linecolors))
                self.actualP = None #reset pointcloud, so as to not accidentally do the same pointcloud twice and wait for the next one from the phone
                self.m.release()
            r.sleep() #wait until next iteration
    def furthest_corners(self, corners, corners_check, plane):
        truefalse_mag=sum(corners_check)
        if truefalse_mag < 2:
            corners_to_check = [True, True]
        perplinedir = np.array([-plane[1], plane[0]], dtype = np.float32)
        pind = -1
        pind2 = -1

        curdot = sys.float_info.min
        curdot2 = sys.float_info.max
        for i in range(len(corners)):
            testdot = np.dot(corners[i][:2], perplinedir)
            if testdot > curdot:
                pind = i
                curdot = testdot
            elif testdot < curdot2:
                pind2 = i
                curdot2 = testdot
        corners_to_check = [pind, pind2]
        for i in range(2):
            if corners_check[corners_to_check[i]]:
                corners_to_check[i] = True
            else:
                corners_to_check[i] = False
        return corners_to_check
    def planevec_intersect(self, u, n):
        pos = [self.position.x, self.position.y, self.position.z]
        wallpos = self.getclosestpoint(n)
        w = [pos[i] - wallpos[i] for i in range(3)]
        numerator = -np.dot(n[:3], w) - n[3] #plane_model[0] * self.position.x + plane_model[1] * self.position.y + plane_model[2] * self.position.z + plane_model[3]
        denominator = np.dot(n[:3], u)#sum([vec[i] * plane_model[i] for i in range(len(vec))])
        s = numerator/denominator
        if s < 0:
            return False
        if s > self.maxcornerdist:
            return True
        p = [w[i] + s*u[i] for i in range(3)]
        #print(p)
        return p

    def isnearpoint(self, point, pcloud):
        for i in pcloud:
            dist = np.linalg.norm([point[j]-i[j] for j in range(3)])
            if dist < self.minimum_non_corner_dist:
                return True
        return False

    def posdiff(self, p, d2 = True):
        if d2:
            val = [self.position.x - p[0], self.position.y-p[1]]
        else:
            val = [self.position.x - p[0], self.position.y-p[1], self.position.z-p[2]]
        return math.sqrt(sum([i**2 for i in val]))

    def pcloud_transform(self, cloud, target_frame, get_points):
        """
        Function to transform point cloud into a desired frame
        """

        try: # Attempt to transform
            if np.mean([p.z for p in cloud.points]) > 10**3: #If data is absolutely ridiculous
                print "ridiculously large numbers found" #print it
                return None, None #return Nones
            newcloud = self.listener.transformPointCloud(target_frame, cloud) #Attempt to transform
            if np.isnan(np.mean([p.z for p in newcloud.points])): #if anything is Not A Number
                print "NAN numbers found" #print it
                return None, None #return Nones
            if get_points: #If code wants points in numpy array form
                points = None
                #points = np.asarray([(p.x, p.y, p.z) for p in newcloud.points], dtype = np.float32) #Create numpy array form of points
            else: #otherwise
                points = None #set to none
        except Exception as inst: #if something goes wrong
            #print inst #print it
            newcloud = None #set information to None
            points = None
        return points, newcloud #return points and cloud

    def getclosestpoint(self, plane):
        """
        Function to get closest point on a plane to the origin in either the odom frame of phone frame depending on the plane_model entered (currently used in odom frame)
        """
        d = plane[3] #d is the distance to the origin from the closest point.
        point = [-i*d for i in plane[0:3]] #Thus the vector of the point from the origin is the negative normal vector, multiplied by the distance.
        return point #return the closest point

    def gettangentpoints(self,plane):
        """
        This function provides end points for a line to draw in rviz of a plane_model.
        """
        point = self.getclosestpoint(plane) #Get the closest point to the origin (this makes sure the part of the plane we draw is in the same area as the other visualized features)
        point[2] = 0; #the second point is set to 0 since the line being drawn in rviz is a line and not a plane.
        perplinedir = [-plane[1], plane[0], 0] #find perpendicular vector
        p1=[self.linelength*perplinedir[i]+point[i] for i in range(3)] #set the first point of the line
        p2=[-self.linelength*perplinedir[i]+point[i] for i in range(3)] #set the second point of the line
        return [Point(*p1), Point(*p2)] #Return list of ROS points

    def PassedPlane(self, plane_to_pass, testplanepoints, position):
        """
        This function checks if one plane is on the other side of another plane compared to a position point.
        """
        plane_norm_vec = np.array(plane_to_pass[0:3], dtype=np.float32, ndmin = 2) #direction vector normal to the savedplane
        closest_point_vec = np.array(self.getclosestpoint(plane_to_pass), dtype = np.float32, ndmin=2) #position vector of a point on the savedplane (particularly the closest in the odom frame)
        position_vec = np.array([position.x, position.y, position.z], dtype = np.float32) # phone position vector
        wall2posvec = position_vec - closest_point_vec #direction vector from the wall to the position
        m = testplanepoints.shape[0] #Number of points in newplane's pointcloud
        testplanevecs = testplanepoints - np.dot(np.ones((m,1)), closest_point_vec) #direction vectors from the savedplane wall to the newplane
        positiondot = np.dot(plane_norm_vec, wall2posvec.T) #dot product for wall to position vector with normal vector of saved plane
        testplanedots = np.dot(plane_norm_vec, testplanevecs.T) #dot product of all the direction vectors to the newplane points with normal vector of the savedplane
        pos_sign = np.sign(positiondot) #Get sign of dot to see which side of savedplane phone is on
        testplane_sign = np.sign(np.average(np.sign(testplanedots))) #Get sign of average of signs of testplanedots to see which side of saved plane the majority of points are on
        if pos_sign-testplane_sign == 0: #if they are on the same side then
            return False #plane not passed
        else: #otherwise
            return True #plane passed
    def wallintersection(self, saved_plane, testplane):
        pass

    def wallvanishing(self, plane, planepoints):
        perplinedir = np.array([-plane[1], plane[0]], dtype = np.float32)
        pind = -1
        pind2 = -1
        curdot = sys.float_info.min
        curdot2 = sys.float_info.max
        for i in range(len(planepoints)):
            testdot = np.dot(planepoints[i]/np.linalg.norm(planepoints[i]), perplinedir)
            if testdot > curdot:
                pind = i
                curdot = testdot
            elif testdot < curdot2:
                pind2 = i
                curdot2 = testdot
        return (planepoints[pind],planepoints[pind2])



if __name__ == '__main__': #Run Code
    node = ClosestWallFinder(True, True) #, True)
    node.run()
