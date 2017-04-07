#!/usr/bin/env python
#coding=utf-8
"""
combine amcl_pose and odom to figureout realtime robot pose in map
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import rospy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Pose
from geometry_msgs.msg import  PoseStamped
import tf


ClockLocker = True
ClockRLocker  = True

class ClearParams:
    def __init__(self):
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')
        rospy.delete_param('~amclodom_frequence')

class AMCL_Odom_Trigger:
    def __init__(self):
        self.define()
        rospy.Subscriber('tf', TFMessage, self.tf_handle)
        rospy.Timer(rospy.Duration(5), self.ClockCB)
        rospy.Timer(rospy.Duration(self.frequency), self.launcher)
        rospy.spin()

    def ClockCB(self, event):
        global ClockLocker
        if not ClockLocker and ClockRLocker:
            ClockLocker = True

    def tf_handle(self, tf_msg):
        global ClockLocker
        global ClockRLocker
        if len(tf_msg.transforms) > 0:
            frame_id = [i.header.frame_id for i in tf_msg.transforms]
            if '/map' in frame_id or 'map' in frame_id:
                if ClockRLocker:
                    rospy.loginfo('amcl_odom: Detect map frame')
                    ClockRLocker = False
            else:
                if ClockLocker:
                    ClockLocker = False
                    rospy.logwarn('amcl_odom: Lose map frame')

    def launcher(self, event):
        if not ClockRLocker:
            try:
                now = rospy.Time.now()
                self.listener.waitForTransform(self.target_frame, self.source_frame, now, rospy.Duration(0.001))
                (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, now)
                self.pub_data(trans, rot)
            except:
                try:
                    self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(), rospy.Duration(1))
                except:
                    pass

    def pub_data(self, trans, rot):
        pose = Pose()
        odom = PoseStamped()
        (px, py, pz) = trans
        (qx, qy, qz, qw) = rot
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        odom.pose = pose
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "/map"
        odom_pub = rospy.Publisher(self.use_odom_topic, PoseStamped, queue_size=1)
        odom_pub.publish(odom)

    def define(self):
        if not rospy.has_param("~use_odom_topic"):
            rospy.set_param("~use_odom_topic", "/robot_position_in_map")
        self.use_odom_topic = rospy.get_param("~use_odom_topic")

        if not rospy.has_param("~target_frame"):
            rospy.set_param("~target_frame", "/map")
        self.target_frame = rospy.get_param("~target_frame")

        if not rospy.has_param("~source_frame"):
            rospy.set_param("~source_frame", "/base_footprint")
        self.source_frame = rospy.get_param("~source_frame")

        if not rospy.has_param("~amclodom_frequence"):
            rospy.set_param("~amclodom_frequence", 0.1)
        self.frequency = rospy.get_param("~amclodom_frequence")

        self.listener = tf.TransformListener()

if __name__ == '__main__':
    rospy.init_node('amcl_odom')
    try:
        rospy.loginfo( "initialization system")
        AMCL_Odom_Trigger()
        ClearParams()
        rospy.loginfo( "process done and quit")
    except rospy.ROSInterruptException:
        rospy.loginfo("follower node terminated.")
