#!/usr/bin/env python
#coding:utf-8
"""
小于阀值点发出报警
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
import rospy
import numpy
import PyKDL
import maplib
import copy
import tf
import random
import collections
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseArray

global_map = OccupancyGrid()
projection_points = collections.deque(maxlen=1)

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_warning_marker_topic')
        rospy.delete_param('~use_stop_flag_topic')
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~use_odom_topic')
        rospy.delete_param('~use_scan_topic')
        rospy.delete_param('~use_projection_topic')

        rospy.delete_param('~particles')
        rospy.delete_param('~detector_resolution')
        rospy.delete_param('~detector_radius')
        rospy.delete_param('~Maxdetect')
        rospy.delete_param('~Mindetect')

class DetectorStopMove():
   def define(self):
        self.cut_points = []#中分线的上下左右终点
        self.centre_points = []
        self.seq = 0
        self.CameraPose = tf.TransformListener()
        self.obstacles = LaserScan()
  
        #topic names
        if not rospy.has_param('~use_warning_marker_topic'):
         rospy.set_param('~use_warning_marker_topic','/warning_marker')
        self.warning_marker_topic = rospy.get_param('~use_warning_marker_topic')

        if not rospy.has_param('~use_stop_flag_topic'):
         rospy.set_param('~use_stop_flag_topic','/stop_flag')
        self.stop_flag_topic = rospy.get_param('~use_stop_flag_topic')

        if not rospy.has_param('~use_map_topic'):
         rospy.set_param('~use_map_topic','/map')
        self.use_map_topic = rospy.get_param('~use_map_topic')

        if not rospy.has_param('~use_odom_topic'):
         rospy.set_param('~use_odom_topic','/robot_position_in_map')
        self.use_odom_topic = rospy.get_param('~use_odom_topic')

        if not rospy.has_param('~use_scan_topic'):
         rospy.set_param('~use_scan_topic','/scan')
        self.use_scan_topic = rospy.get_param('~use_scan_topic')

        if not rospy.has_param('~use_projection_topic'):
         rospy.set_param('~use_projection_topic','/obstacles/projection')
        self.Projection_topic = rospy.get_param('~use_projection_topic')

        if not rospy.has_param('~particles'):
         rospy.set_param('~particles', 60)
        self.particles = rospy.get_param('~particles')

        #最小地图存储模块
        if not rospy.has_param('~detector_resolution'):
         rospy.set_param('~detector_resolution', 20)
        self.mim_space = rospy.get_param('~detector_resolution')

        #误差允许范围
        if not rospy.has_param('~detector_radius'):
         rospy.set_param('~detector_radius', 0.2)
        self.radius = rospy.get_param('~detector_radius')

        #最远警戒范围
        if not rospy.has_param('~Maxdetect'):
         rospy.set_param('~Maxdetect', 2.8)
        self.Maxdetect = rospy.get_param('~Maxdetect')

        #最近警戒范围
        if not rospy.has_param('~Mindetect'):
         rospy.set_param('~Mindetect', 0.0)
        self.Mindetect = rospy.get_param('~Mindetect')
 
        # setting up topics
        self.warning_marker = rospy.Publisher(self.warning_marker_topic, Marker ,queue_size=1)

        self.stop_flag = rospy.Publisher(self.stop_flag_topic, String ,queue_size=1)

        print 'radiu',self.radius
  
   def __init__(self):
        self.define()
        self.map_data()
        rospy.Subscriber(self.use_scan_topic, LaserScan, self.laser_cb)
        rospy.Subscriber(self.use_odom_topic, PoseStamped, self.odom_cb)
        rospy.spin()

   def map_data(self):
        global global_map
        global_map = rospy.wait_for_message(self.use_map_topic, OccupancyGrid)
        static_data = self.statice_area(copy.deepcopy(global_map))

        #geohash算法区域划分
        width = global_map.info.width
        height = global_map.info.height
        map_origin = global_map.info.origin
        resolution = global_map.info.resolution
        rospy.loginfo('map info:\nwidth: ' + str(width) + '  height: ' + str(height))

        self.geohash(static_data, width, height, map_origin, resolution)

   def statice_area(self, map): #visual_test
        static_data = maplib.get_effective_point(map)[1]
        self.static_area_makers(static_data)
        rospy.loginfo('statice area readed: ' + str(len(static_data)))
        return static_data

   def static_area_makers(self, data):
        color = ColorRGBA()
        scale = Point()
        scale.x = 0.05
        scale.y = 0.05
        color.r = 1.0
        color.g = 1.0
        color.b = 0.0
        color.a = 1.0
        self.points_marker = Marker()
        self.points_marker = self.visual_test(data, Marker.POINTS, color, scale) #data=[point1,point2,point3...]

   #地图区域划分
   def geohash(self, data, width, height, map_origin, resolution):
        (X,Y) = (1,1)
        pose_mean = Point()
        pose_mean.x = (width/2) * resolution * X + map_origin.position.x
        pose_mean.y = (height/2) * resolution * Y + map_origin.position.y

        self._map_divide_store(width, height, data, map_origin.position, pose_mean, (X,Y))

   #地图划分区域存储
   def _map_divide_store(self, width, height, data, map_origin, pose_mean, (X,Y)):
        root = self.create_btree(data, width, height, map_origin, pose_mean, (X,Y))
        rospy.loginfo( 'start loading tree' )
        for i in data:
         self.load(root, i)
        self.root = root
        rospy.loginfo( 'end building tree' )

   #创建树
   def create_btree(self,data, width, height, map_origin, pose_mean, (X,Y)):
        root = {'map':[data], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
        rospy.loginfo( 'start building tree' )
        for i in data:
         self.insert(root, i, width, height, map_origin, (X,Y))
        rospy.loginfo( 'end building tree' )
        return root

   #存入值
   def insert(self, root, data, width, height, map_origin, (X,Y)):
        global global_map
        pose_mean = Point()
        pose_mean.x = abs((width/2) * global_map.info.resolution)*X+map_origin.x
        pose_mean.y = abs((height/2) * global_map.info.resolution)*Y+map_origin.y

        linex_start = Point()
        linex_end = Point()
        liney_start = Point()
        liney_end = Point()

        linex_start.x = pose_mean.x
        linex_start.y = map_origin.y
        linex_end.x = pose_mean.x
        linex_end.y = map_origin.y + global_map.info.resolution*height*Y

        liney_start.y = pose_mean.y
        liney_start.x = map_origin.x
        liney_end.y = pose_mean.y
        liney_end.x = map_origin.x + global_map.info.resolution*width*X

        line = [linex_start,linex_end,liney_start,liney_end]
        self.cut_points += line
        self.centre_points.append(pose_mean)

        if width <= self.mim_space or height <= self.mim_space:
         pass
   
        else:
         #print 'width, height: ', width, height
         if data.x >= pose_mean.x and data.y >= pose_mean.y:#EN 的条件
          if root['EN'] != None:
           pass
          elif root['EN'] == None:
           root['EN'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
          else:
           rospy.loginfo( 'create tree EN errro' )
          (X,Y) = (1,1)
          width /= 2
          height /= 2
          self.insert(root['EN'], data, width, height, pose_mean, (X,Y))

         elif data.x >= pose_mean.x and data.y < pose_mean.y:#ES 的条件
          if root['ES'] != None:
           pass
          elif root['ES'] == None:
           root['ES'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
          else:
           rospy.loginfo( 'create tree ES errro' )
          (X,Y) = (1,-1)
          width /= 2
          height /= 2
          self.insert(root['ES'], data, width, height, pose_mean, (X,Y))

         elif data.x < pose_mean.x and data.y >= pose_mean.y:#WN 的条件
          if root['WN'] != None:
           pass
          elif root['WN'] == None:
           root['WN'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
          else:
           rospy.loginfo( 'create tree WN errro' )
          (X,Y) = (-1,1)
          width /= 2
          height /= 2
          self.insert(root['WN'], data, width, height, pose_mean, (X,Y))

         elif data.x < pose_mean.x and data.y < pose_mean.y:#WS 的条件
          if root['WS'] != None:
           pass
          elif root['WS'] == None:
           root['WS'] = {'map':[], 'width': width, 'height': height, 'map_origin': map_origin, 'pose_mean': pose_mean, 'WN': None, 'WS': None, 'EN': None, 'ES': None}
          else:
           rospy.loginfo( 'create tree WS errro' )
          (X,Y) = (-1,-1)
          width /= 2
          height /= 2
          self.insert(root['WS'], data, width, height, pose_mean, (X,Y))

         else:
          rospy.loginfo( 'error during building tree' )

   def load(self,root,i):
        if root['EN'] == None and root['ES'] == None and root['WN'] == None and root['WS'] == None:
        #LEAF
         root['map'].append(i)

      #1
        if root['EN'] != None and root['ES'] == None and root['WN'] == None and root['WS'] == None:
        # EN
         self.load(root['EN'],i)

        if root['EN'] == None and root['ES'] != None and root['WN'] == None and root['WS'] == None:
        # ES
         self.load(root['ES'],i)

        if root['EN'] == None and root['ES'] == None and root['WN'] != None and root['WS'] == None:
        #WN
         self.load(root['WN'],i)

        if root['EN'] == None and root['ES'] == None and root['WN'] == None and root['WS'] != None:
        #WS
         self.load(root['WS'],i)

      #2
        if root['EN'] != None and root['ES'] != None and root['WN'] == None and root['WS'] == None:
         #EN ES
         if root['pose_mean'].x <= i.x:
          if root['pose_mean'].y > i.y:
           self.load(root['ES'],i)
          elif root['pose_mean'].y <= i.y:
           self.load(root['EN'],i)
          else:
           rospy.loginfo('EN ES1 error point not in position')


        if root['EN'] != None and root['ES'] == None and root['WN'] != None and root['WS'] == None:
        # EN WN
         if root['pose_mean'].y <= i.y:
          if root['pose_mean'].x <= i.x:
           self.load(root['EN'],i)
          elif root['pose_mean'].x > i.x:
           self.load(root['WN'],i)
          else:
           rospy.loginfo('EN WN1 error point not in position')


        if root['EN'] != None and root['ES'] == None and root['WN'] == None and root['WS'] != None:
        # EN WS
         if root['pose_mean'].x <= i.x and root['pose_mean'].y < i.y:
          self.load(root['EN'],i)
         elif root['pose_mean'].x > i.x and root['pose_mean'].y > i.y:
          self.load(root['WS'],i)
         else:
          rospy.loginfo('EN WS error point not in position')

        if root['EN'] == None and root['ES'] != None and root['WN'] != None and root['WS'] == None:
        # ES WN
         if root['pose_mean'].x <= i.x and root['pose_mean'].y > i.y:
          self.load(root['ES'],i)
         elif root['pose_mean'].x > i.x and root['pose_mean'].y <= i.y:
          self.load(root['WN'],i)
         else:
          rospy.loginfo('ES WN error point not in position')

        if root['EN'] == None and root['ES'] != None and root['WN'] == None and root['WS'] != None:
        # ES WS
         if root['pose_mean'].y > i.y:
          if root['pose_mean'].x > i.x:
           self.load(root['WS'],i)
          elif root['pose_mean'].x <= i.x:
           self.load(root['ES'],i)
          else:
           rospy.loginfo('ES WS1 error point not in position')


        if root['EN'] == None and root['ES'] == None and root['WN'] != None and root['WS'] != None:
        # WN WS
         if root['pose_mean'].x >= i.x:
          if root['pose_mean'].y <= i.y:
           self.load(root['WN'],i)
          elif root['pose_mean'].y > i.y:
           self.load(root['WS'],i)
          else:
           rospy.loginfo('WN WS1 error point not in position')


      #3
        if root['EN'] != None and root['ES'] != None and root['WN'] != None and root['WS'] == None:
        #EN ES WN
         if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
          self.load(root['EN'],i)

         elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
          self.load(root['ES'],i)

         elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
          self.load(root['WN'],i)

         else:
          #rospy.loginfo('EN ES WN error point not in position')
          return None

        if root['EN'] != None and root['ES'] != None and root['WN'] == None and root['WS'] != None:
        # EN ES WS
         if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
          self.load(root['EN'],i)

         elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
          self.load(root['ES'],i)

         elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
          self.load(root['ES'],i)

         else:
          #rospy.loginfo('EN ES WS error point not in position')
          return None

        if root['EN'] != None and root['ES'] == None and root['WN'] != None and root['WS'] != None:
        # EN WN WS
         if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
          self.load(root['EN'],i)

         elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
          self.load(root['WN'],i)

         elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
          self.load(root['WS'],i)

         else:
          #rospy.loginfo('EN WN WS error point not in position')
          return None

        if root['EN'] == None and root['ES'] != None and root['WN'] != None and root['WS'] != None:
        # ES WN WS
         if i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
          self.load(root['ES'],i)

         elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
          self.load(root['WN'],i)

         elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
          self.load(root['WS'],i)

         else:
          #rospy.loginfo('ES WN WS error point not in position')
          return
      #4
        if root['EN'] != None and root['ES'] != None and root['WN'] != None and root['WS'] != None:
         # EN ES WN WS
         if i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
          self.load(root['EN'],i)

         elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
          self.load(root['ES'],i)

         elif i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
          self.load(root['WN'],i)

         elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
          self.load(root['WS'],i)

         else:
          rospy.loginfo('EN ES WN WS error point not in position')
          return

       #查询值

   def read(self, root, i):
        #rospy.loginfo('checking the map')
        if i.x < root['pose_mean'].x and i.y >= root['pose_mean'].y:#WN 的条件
         if root['WN'] == None:
          for j in root['map']:
           conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
           if conditional:
            #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
            return True
           else:
            continue
         else:
          return self.read(root['WN'], i)

        elif i.x < root['pose_mean'].x and i.y < root['pose_mean'].y:#WS 的条件
         if root['WS'] == None:
          for j in root['map']:
           conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
           if conditional:
            #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
            return True
           else:
            continue
         else:
          return self.read(root['WS'], i)

        elif i.x >= root['pose_mean'].x and i.y >= root['pose_mean'].y:#EN 的条件
         if root['EN'] == None:
          for j in root['map']:
           conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3))) < (self.radius, self.radius)
           if conditional:
            #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
            return True
           else:
            continue
         else:
          return self.read(root['EN'], i)

        elif i.x >= root['pose_mean'].x and i.y < root['pose_mean'].y:#ES 的条件
         if root['ES'] == None:
          for j in root['map']:
           conditional = (abs(round(i.x, 3) - round(j.x, 3)),abs(round(i.y, 3) - round(j.y, 3)))<(self.radius, self.radius)
           if conditional:
            #print conditional, self.radius, (abs(round(i.x,3)-round(j.x,3)),abs(round(i.y,3)-round(j.y,3)))
            return True
           else:
            continue
         else:
          return self.read(root['ES'], i)

        else:
         rospy.loginfo('查询值  error point not in position')

   # get odom
   def odom_cb(self, data):
        #print '更新坐标'
        pose = Pose()
        pose = data.pose
        oriation_angle = self.Q2A([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        global projection_points
        if len(projection_points):
            LaserData = projection_points.pop()
            DurationSec = data.header.stamp.secs - LaserData.header.stamp.secs
            DurationNSec = data.header.stamp.nsecs - LaserData.header.stamp.nsecs
            #print DurationSec, DurationNSec
            if DurationSec == 0 and DurationNSec < 10**8:
                #rospy.loginfo('pub time: ' + str(DurationSec) + str(DurationNSec))
                self.hadle_laser_data(LaserData, pose, oriation_angle)
            else:
                #rospy.loginfo('time differ: ' + str(DurationSec) + str(DurationNSec))
                LaserData = None
                self.hadle_laser_data(LaserData, pose, oriation_angle)
        else:
            LaserData = None
            self.hadle_laser_data(LaserData, pose, oriation_angle)

   def laser_cb(self, data):
        global projection_points
        projection_points.append(copy.deepcopy(data))

   def hadle_laser_data(self, data, pose, oriation_angle):
        LaserData = []
        MateData = []
        datanum = 0
        #CastData = []
        if data:
             for i in data.ranges:
              if self.Mindetect < i < self.Maxdetect:
               MateData.append([i, datanum])
              datanum += 1
             #print len(MateData)
             if len(MateData) > self.particles:
              SampleData = random.sample(MateData, int(len(MateData) * 0.7))
              #SampleData = MateData
              for j in SampleData:
               laser_point = Pose()
               angle = (data.angle_min + oriation_angle) + data.angle_increment * j[1]
               #if angle
               costh = numpy.cos(angle)
               sinth = numpy.sin(angle)
               laser_point.position.x = j[0] * costh + pose.position.x
               laser_point.position.y = j[0] * sinth + pose.position.y
               LaserData.append(laser_point)
              self.data_projection(LaserData)

              #result = False
              #result = self.check(LaserData)
             #if result:
              #rospy.loginfo('判断:是在误差许可之内为地图上已知点')
              #self.addFlag()
              #print 'CastData', len(CastData), '\n', CastData
              """CastData = maplib.GradientDes(MateData)

              ProjectionDataSet = maplib.ProjectionPole(CastData, data, [oriation_angle, pose.position.x, pose.position.y])
              OriginPosition = PointStamped()
              OriginPosition.header = data.header
              OriginPosition.point = ProjectionDataSet[0]

              global global_map
              print ProjectionDataSet[1], numpy.ceil(ProjectionDataSet[1] / global_map.info.resolution)

             ProjectionDataSets = PoseArray()
             Castpose = Pose()
             point = Point()
             #MarkCastData = []
             for MCdata in CastData:

              #print 'MCdata', MCdata,MCdata[1], data.angle_increment
              angle = data.angle_min + data.angle_increment * MCdata[1] - oriation_angle
              point.x = MCdata[0] * numpy.cos(angle) + self.pose.position.x
              point.y = MCdata[0] * numpy.sin(angle) + self.pose.position.y
              Castpose.position = point
              #MarkCastData.append(copy.deepcopy(point))
              ProjectionDataSets.poses.append(copy.deepcopy(Castpose))"""
        else:
             self.data_projection(LaserData)

   def data_projection(self, data):
        ProjectData = PoseArray()
        ProjectData.header.seq = self.seq
        ProjectData.header.stamp = rospy.Time.now()
        ProjectData.header.frame_id = '/map'
        ProjectData.poses = copy.deepcopy(data)
        Projection_pub = rospy.Publisher(self.Projection_topic, PoseArray, queue_size=1)
        Projection_pub.publish(ProjectData)
        self.seq += 1

   #检查镭射是否为地图上的静态障碍物（eg，wall door etc.）
   def check(self, data):
        trigger = 0
        for i in data:
         result = self.read(self.root, i.position)
         if result is True:
          trigger += 1
        if trigger >= 0.8*len(data):
         return False
        else:
         return True


# 视觉显示检测结果

   def visual_test(self, data, Type, color, scale):
        if Type == Marker.POINTS:
         #print 'pub POINTS Marker'
         point_marker = Marker()
         point_marker.header.frame_id = '/map'
         point_marker.header.stamp = rospy.Time.now()
         point_marker.ns = 'detector_visual_test'
         point_marker.action = Marker.ADD

         point_marker.id = 0
         point_marker.type = Type
         point_marker.scale.x = scale.x#0.1
         point_marker.scale.y = scale.y#0.1
         point_marker.points = data
         for i in data:
          point_marker.colors.append(color)
         point_marker.lifetime = rospy.Duration(0.1)
         return point_marker

      #plot LINE_LIST
        if Type == Marker.TEXT_VIEW_FACING:
         #details
         flag_marker = Marker()
         flag_marker.type = Type
         flag_marker.header.frame_id='map'
         flag_marker.text = "     WARNING!!!\n OBSTACLE DETECTED!!"
         flag_marker.ns = "WarningFlag"
         flag_marker.color = color
         flag_marker.scale = scale
         flag_marker.header.stamp = rospy.Time.now()
         flag_marker.lifetime = rospy.Duration(0.3)
         flag_marker.pose = data
         return flag_marker

      #plot robot position

   def Q2A(self,quat):
        rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
        return rot.GetRPY()[2]

   def addFlag(self, pose):
        self.flag_makers(pose)
        self.warning_marker.publish(self.flag_marker)
        #print 'add flag', self.flag_marker
  
   def flag_makers(self, data):
        color = ColorRGBA()
        scale = Point()
        pose = Pose()
        scale.x = 0.01
        scale.y = 0.01
        scale.z = 0.2
        pose = data
        pose.position.z += 0.5
        color.r = 1.0
        color.a = 1.0
        self.flag_marker = self.visual_test(pose, Marker.TEXT_VIEW_FACING, color, scale)

if __name__ == '__main__':
     rospy.init_node("DetectorStopMove", anonymous=True)
     try:
      rospy.loginfo ("initialization system")
      DetectorStopMove()
      ClearParams()
      rospy.loginfo ("process done and quit")
     except rospy.ROSInterruptException:
      rospy.loginfo("node terminated.")
