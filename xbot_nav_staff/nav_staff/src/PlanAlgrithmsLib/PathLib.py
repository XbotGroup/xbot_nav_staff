#!/usr/bin/env python
# coding=utf-8

"""
Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import json
from geometry_msgs.msg import PoseStamped

def read_path(file):
    with open(file) as f:
        return json.load(f)

def save_path(data, file): #data=geometry_msgs/PoseStamped[] poses
    save_data = dict()
    with open(file, 'w') as f:
        num = 0
        for i in data:
            save_data[num] = (i.pose.position.x, i.pose.position.y, i.pose.position.z, i.pose.orientation.x, i.pose.orientation.y, i.pose.orientation.z, i.pose.orientation.w)
            num += 1
        json.dump(save_data, f, indent=4, sort_keys=True, separators=(',', ':'))

def get_store_path(data):
        path = []
        for i in range(len(data)):
            pose = PoseStamped()
            pose.pose.position.x = data['%s'%i][0]
            pose.pose.position.y = data['%s'%i][1]
            pose.pose.position.z = data['%s'%i][2]

            pose.pose.orientation.x = data['%s'%i][3]
            pose.pose.orientation.y = data['%s'%i][4]
            pose.pose.orientation.z = data['%s'%i][5]
            pose.pose.orientation.w = data['%s'%i][6]
            path.append(pose)
        return path

