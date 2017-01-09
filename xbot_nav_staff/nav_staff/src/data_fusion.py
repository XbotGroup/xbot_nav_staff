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
import tf
import numpy
import collections

LaserData = collections.deque(maxlen=1)

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_rplidar_topic')
        rospy.delete_param('~use_asus_topic')
        rospy.delete_param('~scan_topic')


        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')
        rospy.delete_param('~fusion_data_frame')

        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~asus_max_range')


class fusion():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.use_rplidar_topic, LaserScan, self.rplidarCB)
        rospy.Subscriber(self.use_asus_topic, LaserScan, self.asusCB)
        rospy.Timer(self.period, self.PubLaserCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~use_rplidar_topic'):
             rospy.set_param('~use_rplidar_topic', '/rplidar_scan')
        self.use_rplidar_topic = rospy.get_param('~use_rplidar_topic')

        if not rospy.has_param('~use_asus_topic'):
             rospy.set_param('~use_asus_topic', '/asus_scan')
        self.use_asus_topic = rospy.get_param('~use_asus_topic')

        if not rospy.has_param('~scan_topic'):
             rospy.set_param('~scan_topic', '/scan')
        self.scan_topic = rospy.get_param('~scan_topic')

        if not rospy.has_param('~target_frame'):
             rospy.set_param('~target_frame', 'laser')
        self.target_frame = rospy.get_param('~target_frame')

        if not rospy.has_param('~source_frame'):
             rospy.set_param('~source_frame', 'camera_depth_frame')
        self.source_frame = rospy.get_param('~source_frame')

        if not rospy.has_param('~fusion_data_frame'):
            rospy.set_param('~fusion_data_frame', 'global_scan')
        self.fusion_data_frame = rospy.get_param('~fusion_data_frame')

        if not rospy.has_param('~PublishFrequency'):
            rospy.set_param('~PublishFrequency', 0.01)
        PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~asus_max_range'):
            rospy.set_param('~asus_max_range', 4.0)
        self.asus_max_range = rospy.get_param('~asus_max_range')

        self.period = rospy.Duration(PublishFrequency)

        self.laser_data = None
        self.asus_data = None
        self.seq = 0
        self.data = None

    def rplidarCB(self, laser_message):
        self.laser_data = laser_message
        self.laser_data.ranges = [i for i in laser_message.ranges]

    def asusCB(self, asus_message):
        self.asus_data = asus_message
        self.asus_data.ranges = [i if i <= self.asus_max_range else numpy.inf for i in asus_message.ranges]
        self.data_fusion()

    def Pub_Data(self, data):
        pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        pub_data.publish(data)

    def PubLaserCB(self, event):
        global LaserData
        if len(LaserData) > 0:
            self.data = LaserData.pop()
            self.Pub_Data(self.data)
        else:
            if self.data != None:
                self.Pub_Data(self.data)
            else:
                rospy.loginfo('wait for laser scan')
                pass

    def data_fusion(self):
        if self.asus_data != None and self.laser_data != None:
            data = LaserScan()
            data.angle_min = self.laser_data.angle_min
            data.angle_max = self.laser_data.angle_max
            data.header = self.laser_data.header
            data.header.frame_id = self.fusion_data_frame
            data.header.stamp = rospy.Time.now()
            data.header.seq = self.seq
            self.seq += 1
            data.time_increment = self.laser_data.time_increment
            data.angle_increment = self.laser_data.angle_increment
            data.scan_time = self.laser_data.scan_time
            data.range_min = self.laser_data.range_min
            data.range_max = self.laser_data.range_max
            data.ranges=[]
            angle = self.laser_data.angle_min
            for i in self.laser_data.ranges:
                if -0.510632932186 <= angle and angle <= 0.510632932186:
                    num = self.asus_data.angle_max
                    for j in self.asus_data.ranges:
                        # print num - angle, num, angle
                        if abs(num - angle) <= self.asus_data.angle_increment:
                            if i > j:
                                i = j
                        num -= self.asus_data.angle_increment
                data.ranges.append(i)
                angle -= self.laser_data.angle_increment
            if len(data.ranges) == len(self.laser_data.ranges):
                global LaserData
                LaserData.append(data)

if __name__ == '__main__':
    rospy.init_node('data_fusion')
    try:
        rospy.loginfo("initialization system")
        fusion()
        ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")