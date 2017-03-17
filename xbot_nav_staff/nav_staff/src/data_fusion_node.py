#!/usr/bin/env python
# coding=utf-8

"""
rplidar&&asus sensor data fusion node

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import data_fusion

if __name__ == '__main__':
    rospy.init_node('Data_Fusion')
    try:
        rospy.loginfo("initialization system")
        data_fusion.fusion()
        data_fusion.ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")