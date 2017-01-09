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

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_rplidar_topic')
        rospy.delete_param('~use_asus_topic')
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')

class fusion():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.use_rplidar_topic, LaserScan, self.rplidarCB)
        rospy.Subscriber(self.use_asus_topic, LaserScan, self.asusCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~use_rplidar_topic'):
             rospy.set_param('~use_rplidar_topic', '/scan_rplidar')
        self.use_rplidar_topic = rospy.get_param('~use_rplidar_topic')

        if not rospy.has_param('~use_asus_topic'):
             rospy.set_param('~use_asus_topic', '/scan_asus')
        self.use_asus_topic = rospy.get_param('~use_asus_topic')

        if not rospy.has_param('~scan_topic'):
             rospy.set_param('~scan_topic', '/scan')
        self.scan_topic = rospy.get_param('~scan_topic')

        if not rospy.has_param('~target_frame'):
             rospy.set_param('~target_frame', 'laser')
        target_frame = rospy.get_param('~target_frame')

        if not rospy.has_param('~source_frame'):
             rospy.set_param('~source_frame', 'camera_depth_frame')
        source_frame = rospy.get_param('~source_frame')

        listener = tf.TransformListener()
        now = rospy.Time.now()
        listener.waitForTransform(target_frame, source_frame, now, rospy.Duration(2.0))
        print '0'
        (self.trans,self.rot)=listener.lookupTransform(target_frame, source_frame, now)
        print '0.5'

    def rplidarCB(self, laser_message):
        print '1'
        print self.trans
        print self.rot
        # self.Pub_Data(laser_message)
        pass

    def asusCB(self, asus_message):
        pass

    def Pub_Data(self, data):
        pub_data = rospy.Publisher(self.scan_topic, LaserScan, queue_size=1)
        pub_data.publish(data)


if __name__ == '__main__':
    rospy.init_node('fixed_plan_maker')
    try:
        rospy.loginfo("initialization system")
        fusion()
        ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")