#!/usr/bin/env python
#coding=utf-8
"""

This programm is tested on kuboki base turtlebot. 
Copyright (c) Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

"""
import numpy
import rospy
import yaml
from nav_msgs.msg import OccupancyGrid 
import Image
import copy
from threading import Lock
import maplib
import getpass
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import MapMetaData
from nav_msgs.srv import *
from geometry_msgs.msg import Quaternion

ModifyElement = list()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~map_path')
        rospy.delete_param('~map_file')
        rospy.delete_param('~frame_id')
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~root_topic')
        rospy.delete_param('~publish_hz')

class grid_map():
 def __init__(self):
  self.define()
  self.init_map = self.ReadPGMMap()
  self.Map =  copy.deepcopy(self.init_map)
  self.start = True
  self.segments = 50

  rospy.Subscriber(self.root_topic+'/projection', PoseArray , self.MessPoses, queue_size=1)
  rospy.Timer(self.period, self.Reload)
  rospy.Timer((self.period * 300), self.Clear)
  self.AMCLMapSever()
  rospy.spin()
 
 def AMCLMapSever(self):
  with self.locker: 
   rospy.Service('/static_map', GetMap, self.mapCallback)
  
 def mapCallback(self, req):
  with self.locker: 
   rospy.loginfo('sending map')
   res = self.Map
   return res

 def MessPoses(self, poses):
  with self.locker:
   if len(poses.poses) > 0:
    #self.Map.data = copy.deepcopy(self.init_map.data)
    #print 'MessPoses'
    for pose in poses.poses:
     num = maplib.position_num(self.init_map, pose.position)
     self.Map.data[num] += 40
     if self.Map.data[num] >= 90:
      self.Map.data[num] = 100
     global ModifyElement
     if num not in ModifyElement:
      ModifyElement.append(num)
   self.map_pub.publish(self.Map)
   #rospy.loginfo('projection map loaded')

 def Reload(self, event):
  with self.locker:
   #self.Map.data = copy.deepcopy(self.init_map.data)
   self.Map = self._map_(copy.deepcopy(self.Map))
   self.map_pub.publish(self.Map)
   #rospy.loginfo ('update map')
   self.PubMetadata()
   
 def Clear(self,event):
  with self.locker:
   if self.start:
    self.map_pub.publish(self.init_map)
    self.start = False
    rospy.loginfo('init map sended')
   else:
    self.Map = self._map_(copy.deepcopy(self.Map))
    self.map_pub.publish(self.Map)
   #print 'ModifyElement', ModifyElement, len(self.Map.data)
   self.PubMetadata()
   #print maplib.get_effective_point(self.Map)[1]

 def _map_(self, map):
  global ModifyElement
  for i in ModifyElement:
   if map.data[i] > 0 and self.init_map.data[i] != 100:
    map.data[i] -= 10
    if map.data[i] <= 0:
     ModifyElement.remove(i)
     map.data[i] = 0
  return map

 def PubMetadata(self): 
  self.map_metadata.publish(self.Map.info)
  
 def define(self):
  self.locker = Lock()
  countname = getpass.getuser()
  workspace = 'turtlebot_demo'
  self.filepath = '/home/%s/%s/src/nav_staff/map/' % (countname, workspace)

  if not rospy.has_param('~map_path'):
   rospy.set_param('~map_path', self.filepath)
  
  if not rospy.has_param('~map_file'):
   rospy.set_param('~map_file','5-12.yaml')

  if not rospy.has_param('~frame_id'):
   rospy.set_param('~frame_id','map')

  if not rospy.has_param('~use_map_topic'):
   rospy.set_param('~use_map_topic','map_raw')

  if not rospy.has_param('~root_topic'):
   rospy.set_param('~root_topic','/test_obstacles')

  if not rospy.has_param('~publish_hz'):
   rospy.set_param('~publish_hz', 0.001)
  publish_hz = rospy.get_param('~publish_hz')

  self.period = rospy.Duration(publish_hz)
  self.origin_orientation = Quaternion()
  self.origin_orientation.w = -1
  
  #self.radiu = numpy.inf
  self.root_topic = rospy.get_param('~root_topic')
  self.filepath = rospy.get_param('~map_path')
  self.filename = rospy.get_param('~map_file')
  self.frame_id = rospy.get_param('~frame_id')
  self.MapTopic = rospy.get_param('~use_map_topic')
  
  self.map_pub=rospy.Publisher(self.MapTopic, OccupancyGrid ,queue_size=1)
  self.map_metadata=rospy.Publisher("/map_metadata", MapMetaData ,queue_size=1)
  
 def ReadPGMMap(self):
  (self.image, self.resolution, self.origin_position, self.reverse, self.occupied_thresh, self.free_thresh, self.load_time) = self.ReadYaml()
  self.occupied_thresh = int(self.occupied_thresh *  255)
  self.free_thresh = int(self.free_thresh * 255)
  
  print self.free_thresh,self.occupied_thresh 
  
  f=Image.open(self.filepath + self.image)
  (width, height) = f.size
  GridMap=numpy.array(f)
  
  Map=OccupancyGrid()
  Map.header.frame_id = self.frame_id
  Map.info.width = width
  Map.info.height = height
  Map.info.resolution = round(self.resolution, 3)
  Map.info.origin.position.x = self.origin_position[0]
  Map.info.origin.position.y = self.origin_position[1]
  Map.info.origin.position.z = self.origin_position[2]
  Map.info.origin.orientation = self.origin_orientation
  rospy.loginfo( 'map height: ' + str(height) + '     map width:' + str(width))
  for i in range(height):
   data = GridMap[height-i-1]
   for j in data:
    resl = j < [self.free_thresh, self.free_thresh, self.free_thresh]
    res = j > [self.occupied_thresh, self.occupied_thresh, self.occupied_thresh]
    if res.all() and not self.reverse:
     Map.data.append(0)
    elif resl.all() and not self.reverse:
     Map.data.append(100)  
    else:
     print 1
     Map.data.append(50)
  #print Map.info
  rospy.loginfo( 'Map readed' )
  return Map
   
 def ReadYaml(self):
  try:
   with open(self.filepath + self.filename, 'rb') as f:
    data = yaml.load(f)
  except:
   rospy.set_param('~map_path',self.filepath)
   rospy.set_param('~map_file','5-12.yaml')
   self.ReadYaml()
  image = data['image']
  resolution = data['resolution']
  origin = data['origin']
  reverse = data['negate']
  occupied_thresh = data['occupied_thresh']
  free_thresh = data['free_thresh']
  load_time = rospy.Time.now()
  return (image, resolution, origin, reverse, occupied_thresh, free_thresh, load_time)
  
   
if __name__=='__main__':
 rospy.init_node('grid_map')
 try:
  rospy.loginfo ("initialization system")
  grid_map()
  ClearParams()
  rospy.loginfo ("process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("robot twist node terminated.")
