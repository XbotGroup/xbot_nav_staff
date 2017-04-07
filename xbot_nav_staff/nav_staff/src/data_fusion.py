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

# LaserData = collections.deque(maxlen=1)

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
                listener.waitForTransform('camera_depth_frame', 'laser', now, rospy.Duration(1))
                (self.trans, rot) = listener.lookupTransform('camera_depth_frame', 'laser', now)
                rospy.loginfo('get transform')
                break
            except:
                now = rospy.Time.now()
                listener.waitForTransform('camera_depth_frame', 'laser', now, rospy.Duration(1))

    def rplidarCB(self, laser_message):
        self.laser_data = laser_message
        self.Pub_Data(Data.data_transform(self.asus_data,self.laser_data,self.trans))

    def asusCB(self, asus_message):
        self.asus_data = asus_message

    def Pub_Data(self, ranges):
        data = self.laser_data
        data.header.stamp = rospy.Time.now()
        data.header.frame_id = 'global_scan'
        data.header.seq = self.seq
        data.ranges = ranges
        self.seq += 1
        pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        pub_data.publish(data)
