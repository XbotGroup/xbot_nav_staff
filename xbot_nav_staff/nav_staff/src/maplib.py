#!/usr/bin/env python  
#coding=utf-8

""" 
marker's utils tools
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""
#import collections
from geometry_msgs.msg import Point
import PyKDL
import numpy
import copy

 
def GradientDes(data):
 Features = []
 for i in range(len(data)):
  if (len(data)-4)>i>=1:
   SDerivative = round(data[i+3][0] - 3 * data[i+2][0] + 3 * data[i+1][0] - data[i][0], 3)
   NSDerivative = round(data[i+4][0] - 3 * data[i+3][0] + 3 * data[i+2][0] - data[i+1][0], 3)
   PSDerivative = round(data[i+2][0] - 3 * data[i+1][0] + 3 * data[i][0] - data[i-1][0], 3)
   if SDerivative == 0 and NSDerivative * PSDerivative < 0 and PSDerivative > 0:#求凹函数的极致点
    Features.append(data[i])
  elif (len(data)-4)<=i<(len(data)-1):  #if data[i+3] data[i+4] data[i-1]not exist  
   PDerivative =  round(data[i+1][0] - data[i][0], 3)
   if PDerivative == 0:
    Features.append(data[i])
  else:
   Features.append(data[i])
 return Features
 

"""def ProjectionCircle(data, info, origin):
 PoleData = []
 point = Point()
 originP = Point()
 ProjectionData = Point()
 ProjectionDataSet = []
 for j in data:
  angle = info.angle_min + info.angle_increment * j[1] - origin[0]
  point.x = j[0] * numpy.cos(angle) + origin[1]
  point.y = j[0] * numpy.sin(angle) + origin[2]
  PoleData.append(copy.deepcopy(point))
 radius = numpy.sqrt((PoleData[1].x - PoleData[0].x) ** 2 + (PoleData[1].y - PoleData[0].y) ** 2) / 2
 originP.x = (PoleData[3].x - PoleData[2].x) / 2 + PoleData[2].x
 originP.y = (PoleData[3].y - PoleData[2].y) / 2 + PoleData[2].y
 segments = 50
 unit = 2 * numpy.pi / segments 
 for i in range(segments):
  ProjectionData.x = originP.x + r * numpy.cos(unit * i)
  ProjectionData.y = originP.y + r * numpy.sin(unit * i)
  ProjectionDataSet.append(copy.deepcopy(ProjectionData))
 return ProjectionDataSet"""
 
"""def ProjectionPole(data, info, origin):
 ## data = [Mindistance, Maxdistance, StartData, EndData]
 ## Mindistance = [distance, count]
 ## angle = angle_min + angle_increment * count - oriation_angle
 PoleData = []
 point = Point()
 #OriginPosition = Point()
 for j in data:
  #print j[1]
  angle = info.angle_min + info.angle_increment * j[1] - origin[0]
  point.x = j[0] * numpy.cos(angle) + origin[1]
  point.y = j[0] * numpy.sin(angle) + origin[2]
  PoleData.append(copy.deepcopy(point))
 radius = numpy.sqrt((PoleData[1].x - PoleData[0].x) ** 2 + (PoleData[1].y - PoleData[0].y) ** 2)# / 2
 
 #OriginPosition.x = (PoleData[3].x - PoleData[2].x) / 2 + PoleData[2].x
 #OriginPosition.y = (PoleData[3].y - PoleData[2].y) / 2 + PoleData[2].y
 #return [OriginPosition, radius]
 return [PoleData, radius]"""
 
 
 ##转化坐标系
def Trans(quaterion, Mapdata):
 theata = quat_to_angle([quaterion.x,quaterion.y,quaterion.z,quaterion.w])
 Tdata = []
 Tpoint=Point()
 for i in Mapdata:
  Tpoint.x = i.x * numpy.cos(theata) + i.y * numpy.sin(theata)
  Tpoint.y = i.y * numpy.cos(theata) - i.x * numpy.sin(theata)
  Tdata.append(copy.deepcopy(Tpoint))
 return Tdata

#return RPY angle in rad
def quat_to_angle(quat):
 rot = PyKDL.Rotation.Quaternion(quat[0], quat[1], quat[2], quat[3])
 return rot.GetRPY()[2]

#return (x,y,z,w)
def angle_to_quat(angle):#in rad
 rot = PyKDL.Rotation.RotZ(angle)
 return rot.GetQuaternion()

#返回机器人在地图数据中的data格数[num]
def position_num(data, position):
 centremap_cell=map_center_cell(data.info.origin, data.info.resolution)
 pose_x = centremap_cell[0]+int(round(position.x/data.info.resolution))
 pose_y = centremap_cell[1]+int(round(position.y/data.info.resolution))
 num = pose_y * data.info.width + pose_x
 return num

#返回有效区域的坐标集
def get_effective_point(data):
 map_matrix=map_matrix_ranger(data)
 
 #print map_matrix
 
 clear_area,block_area=[],[]
 width=data.info.width
 height=data.info.height
 resolution=data.info.resolution
 map_origin=data.info.origin

 clear=Point()
 block=Point()
 
 centremap_cell=map_center_cell(map_origin,resolution)
 #print centremap_cell
 
 for y in range(height):
  for x in range(width):
   if map_matrix[y][x]==0:
    clear.x=(x-centremap_cell[0])*resolution
    clear.y=(y-centremap_cell[1])*resolution + resolution
    clear_area.append(clear)
   if map_matrix[y][x]==100:
    block.x=(x-centremap_cell[0])*resolution
    block.y=(y-centremap_cell[1])*resolution + resolution
    block_area.append(block)
    #print block,'\n'
    #print block_area

   clear=Point()
   block=Point()
 #print 'block_area',len(block_area)
 return [clear_area,block_area]
 
#返回地图矩阵
def map_matrix_ranger(data):
 height=data.info.height
 width=data.info.width
 #map_matrix=collections.deque()
 map_matrix=[]
 for i in range(height):
  map_width=[]
  #map_width=collections.deque()
  for j in range(width):
   map_width.append(data.data[j+width*i])
  map_matrix.append(map_width)
 return map_matrix
 
 #cell(0,0) in matrixs frame
def map_center_cell(map_origin,resolution):
 centremap_cell=[0-int(round(map_origin.position.x/resolution)),0-int(round(map_origin.position.y/resolution))]
 return centremap_cell

 
