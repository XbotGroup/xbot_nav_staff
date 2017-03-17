#!/usr/bin/env python  
#coding=utf-8

""" 
marker's utils tools
Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.
This program is free software; you can redistribute it and/or modify
This programm is tested on kuboki base turtlebot. 
"""

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import rospy
import numpy
import copy
import PyKDL

#返回有效区域的坐标集
def get_effective_point(data):
    map_matrix = [i for i in data.data]
    block_area = []
    block=Point()
    for y in range(data.info.height):
        for x in range(data.info.width):
            if map_matrix[y * data.info.width + x] == 100:
                block.x = x*data.info.resolution + data.info.origin.position.x
                block.y = y*data.info.resolution + data.info.origin.position.y
                block_area.append(block)
            block=Point()
    return block_area


# 返回有效区域的坐标集
def get_clear_point(data):
    map_matrix = [i for i in data.data]
    clear_area = []
    clear=Point()
    for y in range(data.info.height):
        for x in range(data.info.width):
            if map_matrix[y * data.info.width + x] == 0:
                clear.x = x*data.info.resolution + data.info.origin.position.x
                clear.y = y*data.info.resolution + data.info.origin.position.y
                clear_area.append(clear)
            clear=Point()
    return clear_area

#视觉显示数据
def visual_test(data, Type, color, scale, duration = 1):  # data=[point1,point2,point3...]###################visual_test
    # plot POINTS
    # print len(data),data[0],data[1]
    if Type == Marker.POINTS:
        # print 'pub POINTS Marker'
        point_marker = Marker()
        point_marker.header.frame_id = '/map'
        point_marker.header.stamp = rospy.Time.now()
        point_marker.ns = 'detector_visual_test'
        point_marker.action = Marker.ADD

        point_marker.id = 0
        point_marker.type = Type
        point_marker.scale.x = scale.x  # 0.1
        point_marker.scale.y = scale.y  # 0.1

        point_marker.points = data
        [point_marker.colors.append(color) for i in data]
        point_marker.lifetime = rospy.Duration(duration)
        return point_marker

    # plot LINE_LIST
    if Type == Marker.LINE_LIST:
        # print 'pub LINE_LIST Marker'
        line_marker = Marker()
        line_marker.header.frame_id = '/map'
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = 'detector_visual_test'
        line_marker.action = Marker.ADD

        line_marker.id = 1
        line_marker.type = Type
        line_marker.scale.x = scale.x  # 0.05
        line_marker.scale.y = scale.y  # 0.05

        line_marker.points = data
        for i in data:
            line_marker.colors.append(color)
        line_marker.lifetime = rospy.Duration(duration)
        return line_marker

    # plot texting
    if Type == Marker.TEXT_VIEW_FACING:
        # details
        flag_marker = Marker()
        flag_marker.color = color
        flag_marker.scale = scale
        flag_marker.type = Type
        flag_marker.header.frame_id = 'map'
        flag_marker.text = "test"
        # flag_marker.ns = ""
        flag_marker.header.stamp = rospy.Time.now()
        flag_marker.lifetime = rospy.Duration(duration)
        flag_marker.pose = data
        flag_marker.pose.position.z = 1
        return flag_marker

#地图数据中的data格数[num]
def position_num(map, position):
    pose_x = int(round((position.x - map.info.origin.position.x)/map.info.resolution))
    pose_y = int(round((position.y - map.info.origin.position.y)/map.info.resolution))
    num = pose_y * map.info.width + pose_x
    return num

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