#!/usr/bin/env python
# coding=utf-8

"""
博物馆走走停停node

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
import stop_run

if __name__=='__main__':
     rospy.init_node('StopRun')
     try:
         rospy.loginfo("StopRun: initialization system")
         stop_run.StopRun()
         stop_run.ClearParams()
         rospy.loginfo("StopRun: process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("StopRun: node terminated.")
