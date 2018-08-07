#!/usr/bin/env python

from dialog_iot import FoshWrapper
import time
import struct
import rospy
from geometry_msgs.msg import Quaternion, QuaternionStamped
from std_msgs.msg import Header
import tf
from tf.transformations import quaternion_matrix
import numpy as np


class DongleDriver(object):
    def __init__(self):

        rospy.init_node('dongle_driver')
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('/dongle_orientation', QuaternionStamped, queue_size=10)

        self.fosh = FoshWrapper()

    def run(self):
        devices = self.fosh.find(connect = False)

        try:
            self.fosh.connect('80:EA:CA:00:D3:4A')
            print("Success")
        except Exception as e:
            print(e) #error time :D
            exit()

        config = self.fosh.getConfig()

        #if config is not equal to the fosh.config just send it to the device
        if config != self.fosh.config:
            self.fosh.setConfig()    #set config and also store this configuration in eeprom
            #fosh.setConfig(False) #set config without storing it in eeprom

        #now we wants to get accelerometer data and response will be in f
        self.fosh.subscribe('sensorFusion', self.sensor_fusion_data_callback)
        #send command for start!!!
        self.fosh.start()

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    def sensor_fusion_data_callback(self, handle, data):

        s = struct.Struct('<BBBhhhh')
        readings = s.unpack(data)[3:]

        total_bytes = float(2**15)

        qw = readings[0] / total_bytes
        qx = readings[1] / total_bytes
        qy = readings[2] / total_bytes
        qz = readings[3] / total_bytes
        msg = QuaternionStamped(header=Header(stamp=rospy.Time.now(),frame_id="odom"),
                          quaternion=Quaternion(x=qx, y=qy, z=qz, w=qw))

        matrix = quaternion_matrix([qx ,qy, qz, qw])


        self.pub.publish(msg)
        # TODO: may want ot remove this transform as it doesn't really
        # correspond to a coordinate frame in a physical sense
        self.br.sendTransform((0, 0, 0),
                         (qx, qy, qz, qw),
                         rospy.Time.now(),
                         "dongle",
                         "odom")


if __name__ == '__main__':
    node = DongleDriver()
    node.run()