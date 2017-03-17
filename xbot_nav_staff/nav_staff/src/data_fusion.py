#!/usr/bin/env python
# coding=utf-8

"""
rplidar&&asus sensor data fusion

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from sensor_msgs.msg import LaserScan
import numpy
import collections
from PlanAlgrithmsLib import Data
import tf

LaserData = collections.deque(maxlen=1)

class ClearParams:
    def __init__(self):
        rospy.delete_param('~fussion_use_rplidar_topic')
        rospy.delete_param('~fussion_use_asus_topic')
        rospy.delete_param('~fussion_scan_topic')

        rospy.delete_param('~fussion_PublishFrequency')
        rospy.delete_param('~fussion_asus_max_range')


class fusion():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.use_rplidar_topic, LaserScan, self.rplidarCB)
        rospy.Subscriber(self.use_asus_topic, LaserScan, self.asusCB)
        rospy.Timer(rospy.Duration(self.period), self.PubLaserCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~fussion_use_rplidar_topic'):
             rospy.set_param('~fussion_use_rplidar_topic', '/rplidar_scan')
        self.use_rplidar_topic = rospy.get_param('~fussion_use_rplidar_topic')

        if not rospy.has_param('~fussion_use_asus_topic'):
             rospy.set_param('~fussion_use_asus_topic', '/asus_scan')
        self.use_asus_topic = rospy.get_param('~fussion_use_asus_topic')

        if not rospy.has_param('~fussion_scan_topic'):
             rospy.set_param('~fussion_scan_topic', '/scan')
        self.scan_topic = rospy.get_param('~fussion_scan_topic')

        if not rospy.has_param('~fussion_PublishFrequency'):
            rospy.set_param('~fussion_PublishFrequency', 0.01)
        self.period = rospy.get_param('~fussion_PublishFrequency')

        if not rospy.has_param('~fussion_asus_max_range'):
            rospy.set_param('~fussion_asus_max_range', 4.0)
        self.asus_max_range = rospy.get_param('~fussion_asus_max_range')

        self.laser_data = None
        self.asus_data = None
        self.seq = 0
        self.data = None
        listener = tf.TransformListener()
        while True:
            try:
                now = rospy.Time.now()
                listener.waitForTransform('/laser', '/camera_depth_frame', now, rospy.Duration(0.001))
                (trans, rot) = listener.lookupTransform('/laser', '/camera_depth_frame', now)
                break
            except:
                now = rospy.Time.now()
                listener.waitForTransform('/laser', '/camera_depth_frame', now, rospy.Duration(0.001))
        self.tf_rplidar_asus = trans

    def rplidarCB(self, laser_message):
        self.laser_data = laser_message
        self.laser_data.ranges = [i for i in laser_message.ranges]

    def asusCB(self, asus_message):
        self.asus_data = asus_message
        self.asus_data.ranges = [i if i <= self.asus_max_range else numpy.inf for i in asus_message.ranges]

    def Pub_Data(self, data):
        pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'global_scan'
        data.header.seq = self.seq
        self.seq += 1
        pub_data.publish(data)

    def PubLaserCB(self, event):
        global LaserData
        self.data = None
        if self.asus_data and self.laser_data:
            if self.asus_data.header.stamp.secs - self.laser_data.header.stamp.secs == 0.0 and self.asus_data.header.stamp.nsecs - self.laser_data.header.stamp.nsecs <= 10.0**6:
                Data.data_fusion(self.asus_data, self.laser_data, LaserData, self.tf_rplidar_asus)
                if len(LaserData) > 0:
                    self.data = LaserData.pop()
                    self.Pub_Data(self.data)
                else:
                    if self.data != None:
                        self.Pub_Data(self.data)