#!/usr/bin/env python
# coding=utf-8
"""
costplan map

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy
import time
from threading import Lock
import collections
from geometry_msgs.msg import PoseArray
from PlanAlgrithmsLib import maplib
from nav_msgs.srv import *


ModifyElement = list()
init = False

class ClearParams:
    def __init__(self):
        rospy.delete_param('~obstacle_thread')
        rospy.delete_param('~root_topic')
        rospy.delete_param('~devergency_scale')
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~use_plan_map_topic')

class CostPlanMap():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.root_topic + '/projection', PoseArray, self.ReBuildMapCB, queue_size=1)
        rospy.Timer(self.period, self.PubCB)
        # rospy.Timer((self.period * 30), self.Clear)
        self.AMCLMapSever()
        rospy.spin()

    def define(self):
        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 80)
        self.obstacle_thread = rospy.get_param('~obstacle_thread')

        if not rospy.has_param('~root_topic'):
            rospy.set_param('~root_topic', '/test_obstacles')
        self.root_topic = rospy.get_param('~root_topic')

        if not rospy.has_param('~devergency_scale'):
            rospy.set_param('~devergency_scale', 5)
        self.devergency_scale = rospy.get_param('~devergency_scale')

        if not rospy.has_param('~use_map_topic'):
            rospy.set_param('~use_map_topic', '/map')
        use_map_topic = rospy.get_param('~use_map_topic')

        if not rospy.has_param('~use_plan_map_topic'):
            rospy.set_param('~use_plan_map_topic', '/cost_plan_map')
        self.pub_map_topic = rospy.get_param('~use_plan_map_topic')

        self.OBSTACLE = 100
        self.obstacle_scale = self.devergency_scale #int(self.devergency_scale / 2)
        self.period = rospy.Duration(0.1)
        self.seq = 0
        self.pub_map = OccupancyGrid()
        self.locker = Lock()
        self.Pubdata = None

        self.init_map = rospy.wait_for_message(use_map_topic, OccupancyGrid)
        # print 'get init map'
        self.mapinfo = self.init_map.info
        self.JPS_map_init = []
        rospy.loginfo('devergency_scale: ' + str(self.devergency_scale))
        self.generate_map(self.init_map)

    def generate_map(self, map_message):
        JPS_map_init = self.devergency(map_message.data)
        self.JPS_map_init = [i for i in JPS_map_init]
        # self.Pubdata.append(JPS_map_init)
        self.Pubdata = JPS_map_init
        rospy.loginfo('Generate Init map')
        global init
        init = True

    def devergency(self, map_message):
        map_ = [i for i in map_message]
        for num in range(len(map_message)):
            j = num % self.mapinfo.width
            i = num / self.mapinfo.width
            if map_message[num] == self.OBSTACLE:
                for n in range(self.devergency_scale)[1:]:
                    xp = j + n <= self.mapinfo.width - 1
                    xn = j - n >= 0
                    yp = i + n <= self.mapinfo.height - 1
                    yn = i - n >= 0
                    if xp and xn and yp and yn:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    elif not xn and xp and yn and yp:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j + n] = self.obstacle_thread

                    elif not xp and xn and yn and yp:
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    elif not yn and yp and xn and xp:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    elif not yp and yn and xn and xp:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    elif not xn and not yp and xp and yn:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j + n] = self.obstacle_thread

                    elif not xn and not yn and xp and yp:
                        if map_[i*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j + n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j + n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j + n] = self.obstacle_thread

                    elif not xp and not yn and xn and yp:
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i + n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i + n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    elif not yp and not xp and xn and yn:
                        if map_[i*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[i*self.mapinfo.width + j - n] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j] = self.obstacle_thread
                        if map_[(i - n)*self.mapinfo.width + j - n] < self.obstacle_thread:
                            map_[(i - n)*self.mapinfo.width + j - n] = self.obstacle_thread

                    else:
                        rospy.logerr(' unkown error ')
            else:
                pass
        return map_

    def ReBuildMapCB(self, proj_msg):
        # with self.locker:
        global init
        if init:
            if len(proj_msg.poses) > 0:
                JPS_map = [i for i in self.JPS_map_init]
                for pose in proj_msg.poses:
                    num = maplib.position_num(self.init_map, pose.position)
                    JPS_map[num] = 0
                    for n in range(self.obstacle_scale):
                        JPS_map[num] += 10
                        JPS_map[num+n] += 30
                        JPS_map[num-n] += 30
                        JPS_map[num+n*self.mapinfo.width] += 30
                        JPS_map[num-n*self.mapinfo.width] += 30
                        JPS_map[num+n+n*self.mapinfo.width] += 30
                        JPS_map[num+n-n*self.mapinfo.width] += 30
                        JPS_map[num-n+n*self.mapinfo.width] += 30
                        JPS_map[num-n-n*self.mapinfo.width] += 30
                        if JPS_map[num] >= 100:
                            JPS_map[num] = 100
                        if JPS_map[num+n] >= 100:
                            JPS_map[num+n] -= 30
                        if JPS_map[num-n] >= 100:
                            JPS_map[num-n] -= 30
                        if JPS_map[num+n*self.mapinfo.width] >= 100:
                            JPS_map[num+n*self.mapinfo.width] -= 30
                        if JPS_map[num-n*self.mapinfo.width] >= 100:
                            JPS_map[num-n*self.mapinfo.width] -= 30
                        if JPS_map[num+n+n*self.mapinfo.width] >= 100:
                            JPS_map[num+n+n*self.mapinfo.width] -= 30
                        if JPS_map[num+n-n*self.mapinfo.width] >= 100:
                            JPS_map[num+n-n*self.mapinfo.width] -= 30
                        if JPS_map[num-n+n*self.mapinfo.width] >= 100:
                            JPS_map[num-n+n*self.mapinfo.width] -= 30
                        if JPS_map[num-n-n*self.mapinfo.width] >= 100:
                            JPS_map[num-n-n*self.mapinfo.width] -= 30

                        if JPS_map[num] == 100:
                            break
                    global ModifyElement
                    if num not in ModifyElement:
                        ModifyElement.append(num)
                # self.Pubdata.append(JPS_map)
                self.Pubdata = JPS_map
        else:
            rospy.logwarn('waiting for init map')

    def PubCB(self, event):
        # with self.locker:
        # if len(self.Pubdata) > 0:
        if self.Pubdata != None:
            self.pub_map = OccupancyGrid()
            self.pub_map.header.stamp = rospy.Time.now()
            self.pub_map.header.seq = self.seq
            self.seq += 1
            self.pub_map.header.frame_id = 'map'
            self.pub_map.info = self.mapinfo
            # data = self.Pubdata.pop()
            data = self.Pubdata
            if len(data) == self.mapinfo.height * self.mapinfo.width:
                self.pub_map.data = data
            else:
                for j in range(self.mapinfo.height):
                    self.pub_map.data.extend(data[j])
            pub = rospy.Publisher(self.pub_map_topic, OccupancyGrid, queue_size=1)
            pub.publish(self.pub_map)
        else:
            pub = rospy.Publisher(self.pub_map_topic, OccupancyGrid, queue_size=1)
            self.pub_map.header.stamp = rospy.Time.now()
            pub.publish(self.pub_map)

    # def Clear(self, event):
    #     with self.locker:
    #         global init
    #         if init:
    #             rospy.loginfo('cleaning map')
    #             self.pub_map = self.Fade(self.pub_map)
    #             self.Pubdata.append(self.JPS_map_init)
    #             pass
    #         else:
    #             rospy.logwarn('waiting for init map')
    #
    # def Fade(self, map_msg):
    #     # print 'fading'
    #     CheckElements = []
    #     global ModifyElement
    #     for num in ModifyElement:
    #         for n in range(self.devergency_scale / 2):
    #             # map_msg.data[num] -= 10
    #             if map_msg.data[num + n] > 0 and self.JPS_map_init[num + n] < self.obstacle_thread:
    #                 if (num + n) not in CheckElements:
    #                     CheckElements.append(num + n)
    #                 map_msg.data[num + n] -= 10
    #                 if map_msg.data[num + n] <= 0:
    #                     map_msg.data[num + n] = 0
    #
    #             if map_msg.data[num - n] > 0 and self.JPS_map_init[num - n] < self.obstacle_thread:
    #                 if (num - n) not in CheckElements:
    #                     CheckElements.append(num - n)
    #                 map_msg.data[num - n] -= 10
    #                 if map_msg.data[num - n] <= 0:
    #                     map_msg.data[num - n] = 0
    #
    #             if map_msg.data[num + n + n * self.mapinfo.width] > 0 and self.JPS_map_init[num + n + n * self.mapinfo.width] < self.obstacle_thread:
    #                 if (num + n + n * self.mapinfo.width) not in CheckElements:
    #                     CheckElements.append(num + n + n * self.mapinfo.width)
    #                 map_msg.data[num + n + n * self.mapinfo.width] -= 10
    #                 if map_msg.data[num + n + n * self.mapinfo.width] <= 0:
    #                     map_msg.data[num + n + n * self.mapinfo.width] = 0
    #
    #             if map_msg.data[num + n - n * self.mapinfo.width] > 0 and self.JPS_map_init[num + n - n * self.mapinfo.width] < self.obstacle_thread:
    #                 if (num + n - n * self.mapinfo.width) not in CheckElements:
    #                     CheckElements.append(num + n - n * self.mapinfo.width)
    #                 map_msg.data[num + n - n * self.mapinfo.width] -= 10
    #                 if map_msg.data[num + n - n * self.mapinfo.width] <= 0:
    #                     map_msg.data[num + n - n * self.mapinfo.width] = 0
    #
    #             if map_msg.data[num - n + n * self.mapinfo.width] > 0 and self.JPS_map_init[num - n + n * self.mapinfo.width] < self.obstacle_thread:
    #                 if (num - n + n * self.mapinfo.width) not in CheckElements:
    #                     CheckElements.append(num - n + n * self.mapinfo.width)
    #                 map_msg.data[num - n + n * self.mapinfo.width] -= 10
    #                 if map_msg.data[num - n + n * self.mapinfo.width] <= 0:
    #                     map_msg.data[num - n + n * self.mapinfo.width] = 0
    #
    #             if map_msg.data[num - n - n * self.mapinfo.width] > 0 and self.JPS_map_init[num - n - n * self.mapinfo.width] < self.obstacle_thread:
    #                 if (num - n - n * self.mapinfo.width) not in CheckElements:
    #                     CheckElements.append(num - n - n * self.mapinfo.width)
    #                 map_msg.data[num - n - n * self.mapinfo.width] -= 10
    #                 if map_msg.data[num - n - n * self.mapinfo.width] <= 0:
    #                     map_msg.data[num - n - n * self.mapinfo.width] = 0
    #
    #         # result = [True if map_msg.data[num]==0 else False for num in CheckElements]
    #         # if False not in result:
    #         ModifyElement.remove(num)
    #         # if map_msg.data[num] == 0:
    #         #     ModifyElement.remove(num)
    #             #map_msg.data[num] = 0
    #     return map_msg

    def AMCLMapSever(self):
        with self.locker:
            rospy.Service('/JPS_map_init', GetMap, self.JPSmapServiceCB)

    def JPSmapServiceCB(self, request):
        rospy.loginfo('sending JPS init map...')
        response = OccupancyGrid()
        # response.data = self.JPS_map_init
        response.data = self.Pubdata
        response.info = self.mapinfo
        response.header = self.init_map.header
        return response

if __name__=='__main__':
     rospy.init_node('costplan_map')
     try:
         rospy.loginfo( "initialization system")
         CostPlanMap()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")