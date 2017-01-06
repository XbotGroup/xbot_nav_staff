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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

class marker():
 def define(self):
  self.marker=Marker()
  self.marker.header.frame_id='/map'
  self.marker.ns='robot_uni_marker'
  self.marker.action = Marker.ADD
  self.marker.id = 0
  self.marker.type=Marker.TEXT_VIEW_FACING
  self.marker.color.r=1.0
  self.marker.color.g=1.0
  self.marker.color.b=0.0
  self.marker.color.a=1.0
  self.marker.scale.x=0.1
  self.marker.scale.y=0.1
  self.marker.scale.z=0.1
  self.marker.lifetime = rospy.Duration(0)

  self.marker_pub=rospy.Publisher("robot_uni_marker", Marker, queue_size=1)

  if not rospy.has_param('~OdomTopic'):
   rospy.set_param('~OdomTopic', '/robot_position_in_map')
  self.OdomTopic = rospy.get_param('~OdomTopic')
  
  if not rospy.has_param('~laber_height'):
   rospy.set_param('~laber_height', 0.5)
  self.height = rospy.get_param('~laber_height')

  if not rospy.has_param('~robot_laber'):
   rospy.set_param('~robot_laber','default_robot')
  self.marker_text = rospy.get_param('~robot_laber')


 def pose_callback(self, data):
  self.marker.header.stamp =rospy.Time.now()
  self.marker.pose=data.pose
  self.marker.pose.position.z = self.height

  self.marker.text = self.marker_text
  self.marker_pub.publish(self.marker)

 def PubcmdCB(self, event):
  robot_orien_pub = rospy.Publisher("/robot_orientation", Marker ,queue_size=1)
  color = ColorRGBA()
  scale = Point()
  scale.x=0.05
  scale.y=0.05
  scale.z=0.05
  color.r=2.0
  color.g=0.0
  color.b=0.0
  color.a=1.0
  point_marker = Marker()
  point_marker.header.frame_id = '/map'
  point_marker.header.stamp = rospy.Time.now()
  point_marker.ns = 'robot_uni_marker'
  point_marker.action = Marker.ADD

  point_marker.id = 0
  point_marker.type = Marker.ARROW
  point_marker.scale.x = scale.x * 5
  point_marker.scale.y = scale.y / 2
  point_marker.scale.z = scale.z / 2
  point_marker.pose = self.marker.pose
  point_marker.pose.position.z = self.height + 0.1
  point_marker.color = color
  point_marker.lifetime = rospy.Duration(0.1)
  robot_orien_pub.publish(point_marker)

 def __init__(self):
  self.define()
  rospy.Subscriber(self.OdomTopic, PoseStamped, self.pose_callback)
  rospy.Timer(rospy.Duration(0.001), self.PubcmdCB)
  rospy.spin()


if __name__=='__main__':
 rospy.init_node('robot_uni_marker')
 try:
  rospy.loginfo ("initialization system")
  marker()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
