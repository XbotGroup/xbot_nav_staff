#!/usr/bin/env python
#coding=utf-8
""" 
this code is used for making one marker in map

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class marker():
 def define(self):
  self.marker=Marker()
  self.marker.color.r=1.0
  self.marker.color.g=0.0
  self.marker.color.b=0.0
  self.marker.color.a=1.0
  self.marker.id = 0
  self.marker.ns='robot_uni_marker'
  self.marker.scale.x=0.1
  self.marker.scale.y=0.1
  self.marker.scale.z=0.1
  self.marker.header.frame_id='map'
  self.marker.type=Marker.TEXT_VIEW_FACING
  self.marker.lifetime = rospy.Duration(0)
  self.marker_pub=rospy.Publisher("robot_uni_marker",Marker,queue_size=1)
  
  if not rospy.has_param('~laber_height'):
   rospy.set_param('~laber_height', 0.5)
  self.height = rospy.get_param('~laber_height')
  #self.height = int(self.height)
  
  if not rospy.has_param('~robot_laber'):
   rospy.set_param('~robot_laber','default_robot')
  self.marker_text = rospy.get_param('~robot_laber')


 def pose_callback(self, data):
  self.marker.header.stamp =rospy.Time.now()
  self.marker.pose=data
  self.marker.pose.position.z = self.height
  

  self.marker.text = self.marker_text
  #self.marker.text= '1' #for testing
  self.marker_pub.publish(self.marker)
   
   
 def odom_callback(self, data):
  self.marker.header.stamp =rospy.Time.now()
  self.marker.pose=data.pose.pose
  self.marker.pose.position.z = self.height
  self.marker.text = self.marker_text
  #self.marker.text= '1' #for testing
  self.marker_pub.publish(self.marker)

 def __init__(self):
  rospy.init_node('robot_uni_marker')
  self.define()
  rospy.Subscriber("turtlebot_position_in_map", Pose, self.pose_callback)
  #rospy.Subscriber("odom", Odometry, self.odom_callback)
  rospy.spin()


if __name__=='__main__':
 try:
  rospy.loginfo ("initialization system")
  marker()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
