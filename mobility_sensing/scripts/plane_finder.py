#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, Pose, Point, Vector3
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import numpy as np
import pcl
import tf
import tf2_ros
from threading import Lock
import time
from std_msgs.msg import Header, ColorRGBA, Float64
from visualization_msgs.msg import Marker
from os import system
from python_pointclouds import pointcloud2_to_array
#import copy
import math
import mobility_games.auditory.audio_controller as ac
from audiolazy import *

class plane_finder(object):
    def __init__(self, visualized):
        self.visualized = visualized
        rospy.init_node('plane_finder') #Initialize Node for Code
        rospy.Subscriber('/point_cloud', PointCloud2, self.process_cloud) #Read from Tango's Point Cloud
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_pose) #Read from Tango's Pose
        #self.rate = 44100;
        #self.s, self.Hz = sHz(self.rate)
        #self.ms = 1e-3*self.s
        #self.player = ac.player() #play the sound.
        self.m = Lock() # Create a locker for multithreading management
        #self.pub = rospy.Publisher('/wall_finder', PointCloud2, queue_size=10) #Be ready to publish pointclouds to plane_finder, cloud_transformed, and plane_lines
        #self.pub_transformed = rospy.Publisher('/cloud_transformed', PointCloud, queue_size=10)
        self.vis_pub = rospy.Publisher('/wall_viz', Marker, queue_size=10)
        self.dis_pub = rospy.Publisher("/wall_dist", Float64, queue_size = 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        #self.listener = tf.TransformListener() #Initialize Transformer class that allows for easy coordinate frame transformations
        self.CurrP = None #Initialize Point List
        self.actualP = None #Initialize Point_Cloud from Tango
        self.resolutionfactor = 5 #Factor by which to lower the resolution of the point cloud (i.e. if it is 10, we only look at every 10th point of the pointcloud.)
        self.planetrynum = 5 #Set Number of Times to try to find a plane before giving up
        self.planeDistThreshold = .03
        #self.abc_match_threshold = .075 #amount of difference there can be between a, b, and c parameters in plane_models without assuming the current wall is different from the previous wall. !!!(unsure about what this number should be)
        #self.d_match_threshold = .2 #amount of difference there can be between d parameter in plane_models without assuming the current wall is different from the previous wall. !!!(unsure about what this number should be)
        self.verticalityThreshold = .3 #Set how vertical walls need to be (the maximum z component of the normal vector of the plane.) !!!(unsure about what this number should be)
        self.t = 0.0 #Set current Rostime
        self.robustThreshold = 400 #Minimum number of points required to find plane
        self.ycutoff = 0; #How much of the phone's view is used (0 uses only the right side of the phone camera, a positive number will allow for more on the left to be seen as well, and a negative number will favor only the most right points)
        #self.saved_plane_model = None; #Initialize saved plane
        self.altsounddist = .5; #distance up to which there is no sound difference
        #self.untransformed_saved_plane_model = None; #Initialize untransformed version of saved plane
        self.position = None; #Initialize current phone location
        self.walldist = 0; #Initialize Distance to Wall
        self.last_sound_time = rospy.Time.now(); #Initialize the last time a sound was made
        self.linelength = 3 #Determines length of drawn line in rviz
        self.linepoints = [] #Initialize line points for wall drawing in rviz
        color = ColorRGBA(.25,.42,.88,1) #Initialize color of the wall drawing in rviz
        self.colors = [color, color] #Initialize gradient of color of wall drawing in rviz

    def process_cloud(self, msg):
        """
        This function processes the incoming point cloud from the phone an lowers the resolution of the cloud in an attempt to run a bit faster.
        This function also uses a lock to make sure that actualP isn't being used in a calculation when the code receives new data from the tango.
        """
        with self.m:
            self.actualP = msg #Receive pointcloud

    def process_pose(self, msg):
        """
        This function receives the tango position.
        It also uses a lock in order to make sure the position isn't being used in a calculation when it is updated.
        """
        with self.m:
            self.position = msg.pose.position; #Get position of Tango

    def plane_distance(self, saved_plane_model, position):
        """
        This function uses a formulat to find the distance to a plane_model in the odom reference frame to the phone's position (also in the odom reference frame)
        the equation used is abs(a*x + b*y + c*z + d) / sqrt(a^2 + b^2 + c^2) where a, b, c, and d are the plane_model in the form (ax+by+cz+d = 0) and x, y, and z are the position of the phone
        """
        newdist = abs(position.x*saved_plane_model[0] + position.y*saved_plane_model[1] + position.z*saved_plane_model[2] + saved_plane_model[3])/math.sqrt(math.pow(saved_plane_model[0],2) + math.pow(saved_plane_model[1],2) + math.pow(saved_plane_model[2],2))
        return newdist

    def CloudList(self, msg):
        return pointcloud2_to_array(msg)


    def run(self):
        """
        This function holds the main loop that continuously finds walls and updates the wall distance
        """
        r = rospy.Rate(10) #Attempts to run at a rate of 10 times a second (although never reaches this speed)
        while not rospy.is_shutdown(): #Start main while loop
            if not self.actualP is None and not self.position is None: #Wait until phone has found a pointcloud and a phone position
                self.m.acquire()# LOCK
                actualPCopy = self.pcloud_transform(self.actualP, 'odom')
                #self.CurrP, actualPCopy = self.pcloud_transform(self.actualP, '/odom', True)
                #^Transform the pointcloud into the odom's reference frame from the depth_camera's frame.  Also returns the points in the form of a ROS Pointcloud and a numpy Array

                if actualPCopy is None: #If something went wrong with the transform (if the transforms haven't been set yet or some timing issues happen)
                    pass #continue loop
                else: #otherwise:
                    self.CurrP = np.squeeze(self.CloudList(actualPCopy))[::self.resolutionfactor]
                    cloud_filtered = pcl.PointCloud(self.CurrP) #Convert points into pcl Point Cloud (different from Ros point cloud)

                    #Find Plane with most points
                    seg = cloud_filtered.make_segmenter() #Set segmenter (tries to find segments in pointcloud)
                    seg.set_optimize_coefficients(True) #Not sure what this is
                    seg.set_model_type(pcl.SACMODEL_PLANE) #Set model that segmenter finds to a plane
                    seg.set_method_type(pcl.SAC_RANSAC) #Set method for finding plane to RANSAC algorithm
                    seg.set_distance_threshold(self.planeDistThreshold) #Not sure what this is
                    #match = False #The variable that determines if the previous plane is the same plane as found now.
                #try: #Start Try statment (a lot can go wrong in the following code from timing issues, to not having enough data to set a plane.  In the end, the try except is just a much easier and probably quicker way to skip the current iteration and move to the next one)
                    for i in range(self.planetrynum): #Continue trying to find planes for self.planetrynum times.
                        #check = False #variable that determines if code should check if the current wall or previous wall is closer.
                        #usesave = None #variable that determines whether or not to use the current wall or the previous wall.
                        indices, plane_model = seg.segment() #Receive plane_model [a, b, c, d] for equation (ax + by + cz + d = 0), also receive indices in the plane
                        #if self.saved_plane_model is not None : #If there is a previous plane
                        #    if (abs(plane_model[0]-self.saved_plane_model[0]) < self.abc_match_threshold and
                        #        abs(plane_model[1]-self.saved_plane_model[1]) < self.abc_match_threshold and
                        #        abs(plane_model[2]-self.saved_plane_model[2]) < self.abc_match_threshold and
                        #        abs(plane_model[3]-self.saved_plane_model[3]) < self.d_match_threshold): #See if current plane is relatively close to previous plane
                        #        match = True #The walls match
                        #    else:
                        #        check = True #The walls don't match and we should find which is closer.
                        if len(indices) < self.robustThreshold: #If the wall isn't robust enough
                            print("No Robust Planes") #Print that the wall is bad
                            #if self.saved_plane_model: #if there is a saved_plane_model
                            #    newdist = self.plane_distance(self.saved_plane_model, self.position) #Use saved_plane_model
                            #else: #otherwise
                            #    newdist = None #set distance to none
                            self.linepoints = self.gettangentpoints(plane_model) #Find points for line to draw in rviz
                            #self.walldist = newdist #Set walldistance
                            break #skip to next iteration of while loop
                        if abs(plane_model[2]) < self.verticalityThreshold: #If the plane is vertical enough or the plane matches !!!(the "or match" may allow for finding a vertical wall that slopes into something horizontal.)
                            #self.cloud_plane = cloud_filtered.extract(indices, negative=False) #Extract plane indices.
                            #self.planeDistThreshold
                            #actualPCopy.points  = [actualPCopy.points[indx] for indx in indices] #Set ROS pointcloud to only include plane points
                            new_plane_dist= self.plane_distance(plane_model, self.position)
                            #untransformed_CurrP, actualPCopy = self.pcloud_transform(actualPCopy, "/depth_camera", True) #Transform back into depth_camera frame from odom frame
                            #untransformed_plane_cloud = pcl.PointCloud(untransformed_CurrP) #create pcl pointcloud for depth_camera frame pointcloud
                            #FIND PLANE AGAINT BUT IN NEW REFERENCE FRAME
                            #seg = untransformed_plane_cloud.make_segmenter()
                            #seg.set_optimize_coefficients(True)
                            #seg.set_model_type(pcl.SACMODEL_PLANE)
                            #seg.set_method_type(pcl.SAC_RANSAC)
                            #seg.set_distance_threshold(self.planeDistThreshold)
                            #new_indices, new_plane_model = seg.segment() #create new plane model !!!!!!!!!!!!!!!(GET RID OF ALL THIS EVENTUALLY, THE ONLY THING I NEED THE PLANE_MODEL FOR IS FINDING THE DISTANCE IN THE NEW REFERENCE FRAME, THIS PROCESS PROBABLY TAKES MUCH LONGER THAN THE MATH REQUIRED TO FIND THE DISTANCE.)
                            #new_plane_dist= self.plane_distance(plane_model, self.position) #Find distance to newly found plane
                            #if check: #if the current plane and previous plane were different (i.e. if check)

                            #    if abs(new_plane_dist) > self.walldist: #If the old plane was closer
                            #        plist = []
                            #        for ind in indices:
                            #            p = actualPCopy.points[ind]
                            #            plist.append((p.x, p.y, p.z))
                            #        plane_points = np.array(plist, dtype = np.float32)
                            #        if self.PassedPlane(self.saved_plane_model, plane_points, self.position):
                            #            usesave = False
                            #        else:
                            #            usesave = True #Use the save
                            #    else: #otherwise
                            #        usesave = False #Use the current plane
                            #if usesave: #If save is used
                            #    print("SAVE USED") #Print that save is used
                                #if self.saved_plane_model: #If there is a saved plane
                            #    newdist = self.plane_distance(self.saved_plane_model, self.position) #set distance
                                #else:
                                #    newdist = None
                            #    self.linepoints = self.gettangentpoints(self.saved_plane_model) #make line for rviz
                            #    self.walldist = newdist # set walldist
                            #    break #skip to next iteration of while loop

                            #self.walldist = new_plane_dist #Set walldist to new plane distance
                            self.linepoints = self.gettangentpoints(plane_model) #Make line for RVIZ
                            #print "plane model: " + str(plane_model)
                            #self.pub.publish(actualPCopy);

                            #Since the only way to get to this part of the code is if the current plane was used, the code now overrides all of the saved information with the new plane's information
                            self.walldist = new_plane_dist #set walldist to new walldist
                            #self.saved_plane_model = plane_model #set saved plane to the current plane
                            #self.untransformed_saved_plane_model = new_plane_model # set untransformed plane to current untransformed plane

                            break #start next wall find
                        else: #if the plane didn't make the cut, and wasn't a wall
                            self.CurrP = np.delete(self.CurrP, indices, 0)
                            if self.CurrP.shape[0] < 5:
                                break
                            cloud_filtered = pcl.PointCloud(self.CurrP) # set new pcl pointcloud that uses the updated ROS pointcloud (which doesn't have the indices in the nonwall plane that was found)

                            #MAKE NEW SEGMENTER TO BE READY TO FIND THE PLANE
                            seg = cloud_filtered.make_segmenter()
                            seg.set_optimize_coefficients(True)
                            seg.set_model_type(pcl.SACMODEL_PLANE)
                            seg.set_method_type(pcl.SAC_RANSAC)
                            seg.set_distance_threshold(self.planeDistThreshold)
                            continue #try to find another plane, ignore the one previously found
                #except Exception as inst: #If something goes wrong
                    #pass #ignore it
                #    print inst #print it
                    self.dis_pub.publish(self.walldist)
                    print "wall_distance: " + str(self.walldist) #print the current wall distance
                    #SEND MARKER TO RVIZ PUBLISHER
                    if (self.visualized):
                        self.vis_pub.publish(Marker(header=Header(frame_id="odom", stamp=self.actualP.header.stamp),
                                                type=Marker.LINE_LIST,
                                                ns = 'current_plane',
                                                scale=Vector3(x=.1,y=.1,z=.1),
                                                points = self.linepoints,
                                                colors = self.colors))

                    """if rospy.Time.now()-self.last_sound_time > rospy.Duration(.75) and self.walldist is not 0: #If it's been longer than 2 seconds since last sound, and the wall distance is greater than half a meter
                        self.last_sound_time = rospy.Time.now() #reset sound time
                        if abs(self.walldist) < self.altsounddist:
                            system("aplay ../auditory/sound_files/ding.wav")
                        else:
                            freq = max(min(100*math.exp((abs(self.walldist)/2))+330, 1200), 330) #set frequency based on distance
                            #print freq #Print out the frequency
                            self.synth = ac.synth(freq, synth="digitar", fade = 1) #create synth for sound
                            self.synth = ac.delay(self.synth, 3, self.s*.2, .5) #add delay to the synth
                            ac.playstream(self.synth, self.player, seconds = 1)"""


                self.actualP = None #reset pointcloud, so as to not accidentally do the same pointcloud twice and wait for the next one from the phone
                self.walldist = -1
                self.m.release()

            r.sleep() #wait until next iteration

    def pcloud_transform(self, cloud, target_frame):
        """
        Function to transform point cloud into a desired frame
        """

        try:
            source_frame = cloud.header.frame_id
            #now = rospy.Time.now()
            #self.tfBuffer.waitForTransform(target_frame, source_frame, now, rospy.Duration(1.0))
            trans = self.tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time.now(), rospy.Duration(1.0))
            #print(trans)
            #cloud.fields = cloud.fields[:3]
            #print(cloud.fields)
            #(trans, rot) = self.listener.lookupTransform(target_frame, source_frame, now)
            #t = Transform(trans, rot)
            newcloud = do_transform_cloud(cloud, trans)
            #print('worked')
        except (tf2_ros.ExtrapolationException, tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
            print e
            newcloud = None
            #print("transform exception")
        return newcloud #return points and cloud

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
        #print "plane parameters", plane #print the plane parameters
        p1=[self.linelength*perplinedir[i]+point[i] for i in range(3)] #set the first point of the line
        p2=[-self.linelength*perplinedir[i]+point[i] for i in range(3)] #set the second point of the line
        return [Point(*p1), Point(*p2)] #Return list of ROS points

    """def PassedPlane(self, plane_to_pass, testplanepoints, position):
        This function checks if one plane is on the other side of another plane compared to a position point.
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
            return True #plane passed"""



if __name__ == '__main__': #Run Code
    node = plane_finder(True)
    node.run()
