#!/usr/bin/env python
# coding=utf-8
"""
Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

"""
import rospy
import base_controller

if __name__ == "__main__":
    try:
        rospy.init_node('BaseController_X')
        rospy.loginfo("initialization system")
        try:
            base_controller.BaseController()
            base_controller.ClearParams()
        except KeyboardInterrupt:
            base_controller.ClearParams()
        rospy.loginfo("process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("unknown_detector node terminated.")