#!/usr/bin/env python
# coding=utf-8
"""
骨骼识别目标发布

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
import collections
from nav_msgs.msg import OccupancyGrid
import maplib
from threading import Lock

GotGoal = collections.deque()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_skeleton_topic')
        rospy.delete_param('~publish_frequency')
        rospy.delete_param('~pub_topic')
        rospy.delete_param('~use_robotposition_topic')


class Skeleton():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.SkeletonTopic, PointStamped, self.SGCB)
        rospy.Subscriber(self.robotposition, PoseStamped, self.RBCB)
        rospy.Timer(self.period, self.PubcmdCB)
        rospy.spin()

    def SGCB(self, data):
        with self.locker:
            self.goal = data

    def RBCB(self, data):
        with self.locker:
            odom = data
            map = rospy.wait_for_message('/map', OccupancyGrid)
            while not rospy.is_shutdown():
                num = maplib.position_num(map, self.goal.point)
                if not map.data[num]:
                    global GotGoal
                    GotGoal.append(self.goal)
                    break
                else:
                    x = odom.pose.position.x
                    y = odom.pose.position.y
                    if (round(self.goal.point.x - x, 3)):
                        self.goal.point.x += 0.05 * (self.goal.point.x -x)/abs(self.goal.point.x -x)
                        self.goal.point.y += (self.goal.point.y -y)/(self.goal.point.x - x) * self.goal.point.x
                    else:
                        self.goal.point.y += 0.05 * (self.goal.point.y -y)/abs(self.goal.point.y -y)
                    if round(self.goal.point.x - x, 3) == 0 and round(self.goal.point.y - y, 3) ==0 :
                        rospy.loginfo('cannnot find valid goal')
                        break

    def PubcmdCB(self, data):
        global GotGoal
        if len(GotGoal):
            rospy.loginfo('pub skeleton goal')
            goal = GotGoal.pop()
            self.pub.publish(goal)


    def define(self):
        # parameters
        if not rospy.has_param('~use_skeleton_topic'):
            rospy.set_param('~use_skeleton_topic', '/skelecton/clicked_point')
        self.SkeletonTopic = rospy.get_param('~use_skeleton_topic')

        if not rospy.has_param('~publish_frequency'):
            rospy.set_param('~publish_frequency', 3)
        self.frequency = rospy.get_param('~publish_frequency')

        if not rospy.has_param('~pub_topic'):
            rospy.set_param('~pub_topic', '/clicked_point')
        self.pub_topic = rospy.get_param('~pub_topic')

        if not rospy.has_param('~use_robotposition_topic'):
            rospy.set_param('~use_robotposition_topic', '/robot_position_in_map')
        self.robotposition = rospy.get_param('~use_robotposition_topic')

        self.period = rospy.Duration(self.frequency)

        self.pub = rospy.Publisher(self.pub_topic, PointStamped, queue_size=1)

        self.locker = Lock()

if __name__ == '__main__':
    rospy.init_node('Skeleton')
    try:
        rospy.loginfo("initialization system")
        Skeleton()
        ClearParams()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")