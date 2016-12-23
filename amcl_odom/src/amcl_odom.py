#!/usr/bin/env python
#coding=utf-8
"""
combine amcl_pose and odom to figureout realtime robot pose in map
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

import rospy,math,tf
from geometry_msgs.msg import Pose
from geometry_msgs.msg import  PoseStamped

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_odom_topic')
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')

class amcl_odom():
 def __init__(self):
  self.define()
  self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time(), rospy.Duration(2))
  while not rospy.is_shutdown():
   try:
    now = rospy.Time.now()
    self.listener.waitForTransform(self.target_frame, self.source_frame, now, rospy.Duration(0.01))
    (trans,rot)=self.listener.lookupTransform(self.target_frame, self.source_frame, now)
    self.pub_data(trans,rot)
   except:
    continue
  
 def pub_data(self,trans,rot):
  (px,py,pz)=trans
  (qx,qy,qz,qw)=rot
  
  self.pose.position.x=px
  self.pose.position.y=py
  self.pose.position.z=pz
   
  self.pose.orientation.x=qx
  self.pose.orientation.y=qy
  self.pose.orientation.z=qz
  self.pose.orientation.w=qw
   
  self.odom.pose=self.pose
  self.odom.header.stamp=rospy.Time.now()
  self.odom.header.frame_id="map"
   
  self.odom_pub.publish(self.odom)
  
 def define(self):
 
  if not rospy.has_param("~use_odom_topic"):
   rospy.set_param("~use_odom_topic","/robot_position_in_map")
  self.use_odom_topic=rospy.get_param("~use_odom_topic")
  
  if not rospy.has_param("~target_frame"):
   rospy.set_param("~target_frame","/map")
  self.target_frame=rospy.get_param("~target_frame")

  if not rospy.has_param("~source_frame"):
   rospy.set_param("~source_frame","/base_footprint")
  self.source_frame=rospy.get_param("~source_frame")

  self.listener=tf.TransformListener()
  self.odom_pub=rospy.Publisher(self.use_odom_topic, PoseStamped, queue_size=1)
  self.odom=PoseStamped()
  self.pose=Pose()
  
if __name__ == '__main__':
 rospy.init_node('amcl_odom')
 try:
  rospy.loginfo( "initialization system")
  amcl_odom()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")
