#!/usr/bin/env python
# coding=utf-8
"""
laser数据检测近场障碍

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from sensor_msgs.msg import LaserScan
from PlanAlgrithmsLib import maplib

class ClearParams():
    def __init__(self):
        rospy.delete_param("scan_topic")

class laser_detector():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.ScanTopic, LaserScan, self.HandleLaserMessage)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~scan_topic'):
            rospy.set_param('~scan_topic', '/scan')
        self.ScanTopic = rospy.get_param('~scan_topic')
        if not rospy.has_param('~detect_min'):
            rospy.set_param('~detect_min', -3.14)
        self.DetectMin = rospy.get_param('~detect_min')
        if not rospy.has_param('~detect_max'):
            rospy.set_param('~detect_max', 3.14)
        self.DetectMax = rospy.get_param('~detect_max')

    def HandleLaserMessage(self, message):
        data = message.ranges
        min_dis = message.range_min
        angle_start = message.angle_min
        angle_end = message.angle_max
        scan_time = message.header.stamp
        angle_increment = message.angle_increment



if __name__=='__main__':
    rospy.init_node('laser_detector')
    try:
        rospy.loginfo( "initialization system")
        laser_detector()
        ClearParams()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
