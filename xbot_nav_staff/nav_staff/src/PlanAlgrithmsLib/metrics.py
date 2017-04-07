#!/usr/bin/env python
# coding=utf-8
"""
地图获取转换

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

from geometry_msgs.msg import Point
import numpy
import copy

#返回机器人在地图数据中的data格数[num]
def Position_Num(info, position):
    centremap_cell = Map_Center_Cell(info.origin, info.resolution)
    pose_x = centremap_cell[0]+int(round(position.x / info.resolution))
    pose_y = centremap_cell[1]+int(round(position.y / info.resolution))
    num = pose_y * info.width + pose_x
    return num

def Num_Position(info, num):
    position = Point()
    x = num % info.width
    y = num / info.width
    centremap_cell = Map_Center_Cell(info.origin, info.resolution)
    position.x = (x - centremap_cell[0]) * info.resolution
    position.y = (y - centremap_cell[1]) * info.resolution
    return position

 #cell(0,0) in matrixs frame
def Map_Center_Cell(map_origin, resolution):
    centremap_cell = [0 - int(round(map_origin.position.x / resolution)), 0 - int(round(map_origin.position.y / resolution))]
    return centremap_cell

def Empty_Map_Generation(data):
    map_data = numpy.array(copy.deepcopy(data.data))
    width = data.info.width
    height = data.info.height
    map_matrix = map_data.reshape(height, width)
