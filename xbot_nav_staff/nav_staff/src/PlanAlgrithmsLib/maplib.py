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
from visualization_msgs.msg import Marker
import rospy

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

#视觉显示数据
def visual_test(data, Type, color, scale):  # data=[point1,point2,point3...]###################visual_test
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
        for i in data:
            point_marker.colors.append(color)
        point_marker.lifetime = rospy.Duration(0.2)
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
        line_marker.lifetime = rospy.Duration(0.5)
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
        flag_marker.lifetime = rospy.Duration(1)
        flag_marker.pose = data
        flag_marker.pose.position.z = 1
        return flag_marker

#地图数据中的data格数[num]
def position_num(map, position):
    pose_x = int(round((position.x - map.info.origin.position.x)/map.info.resolution))
    pose_y = int(round((position.y - map.info.origin.position.y)/map.info.resolution))
    num = pose_y * map.info.width + pose_x
    return num