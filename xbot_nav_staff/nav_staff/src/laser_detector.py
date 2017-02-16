#!/usr/bin/env python
# coding=utf-8
"""
laser数据检测近场障碍

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy

class ClearParams():
    def __init__(self):
        rospy.delete_param("")

class laser_detector():
    def __init__(self):
        self.define()

    def define(self):

    def


if __name__=='__main__':
    rospy.init_node('laser_detector')
    try:
        rospy.loginfo( "initialization system")
        laser_detector()
        ClearParams()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
