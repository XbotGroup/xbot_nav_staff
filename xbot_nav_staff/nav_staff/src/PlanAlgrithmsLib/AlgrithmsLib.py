#!/usr/bin/env python
# coding=utf-8
"""
plan 算法库

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import numpy
import heapq
import itertools
from geometry_msgs.msg import PoseStamped
import copy
from nav_msgs.srv import *

# init = True


class PriorityQueue():

    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.counter = itertools.count()  # unique sequence count

    def add_task(self, task, priority=0):
        count = next(self.counter)
        entry = [priority, count, task]
        heapq.heappush(self.pq, entry)

    def pop_task(self):
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.pq) == 0

class FoundPath(Exception):
    pass


class JPS():
    ##########################################################################################################
    # JUMP-POINT SEARCH                                                                                      #
    #  Described: https://harablog.wordpress.com/2011/09/07/jump-point-search/, and thanks Christopher Chu   #
    #                                                                                                        #
    # Using this function:                                                                                   #
    #   - use JPS.get_map(map)                                                                               #
    #   - use JPS.get_path(end, start) to get paths. input(end,start)                                        #
    #    end:point, start:piont, map: OccupancyGrid                                                          #
    #                                                                                                        #
    # Note: this implementation allows diagonal movement and "corner cutting"                                #
    #                                                                                                        #
    ##########################################################################################################

    def __init__(self):
        self.define()

    def define(self):
        rospy.loginfo('intial JPS algrithm')
        self.obstacle_thread = 20
        self.UNINITIALIZED = 0
        self.OBSTACLE = 100
        self.ORIGIN = 0
        self.DESTINATION = -2
        self.devergency_scale = 6

        self.Queue = PriorityQueue()

        self.JPS_map = None
        self.start_from = None
        self.end_with = None
        # rospy.wait_for_service('/JPS_map_init')
        # try:
        #     init_map_service = rospy.ServiceProxy('/JPS_map_init', GetMap)
        #     init_map = init_map_service()
        #     self.JPS_map = init_map.map.data
        #     self.mapinfo = init_map.map.info
        #     rospy.loginfo('get JSP init map...')
        # except:
        #     rospy.logwarn('cannot get JPS_map_init service')
        #     pass

    def get_map(self, map_message):
        self.mapinfo = map_message.info
        self.JPS_map = map_message.data
        # rospy.loginfo('received updata map...')

    def get_path(self, end, start):
        rospy.loginfo('starting gernerating plan')
        if self.JPS_map != None:
            self.Queue = PriorityQueue()
            self.start_from = None
            self.end_with = None
            self.start_from = (int((start.x - self.mapinfo.origin.position.x)/ self.mapinfo.resolution) + int((start.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution) * self.mapinfo.width)
            self.end_with = (int((end.x - self.mapinfo.origin.position.x) / self.mapinfo.resolution) + int((end.y - self.mapinfo.origin.position.y) / self.mapinfo.resolution) * self.mapinfo.width)
            if self.JPS_map[self.end_with] >= 50:
                rospy.logwarn('goal is not walkable, unable to generate a plan, dangerous %: ' + str(self.JPS_map[self.end_with]))
                return None
            if self.JPS_map[self.start_from] >= 80:
                rospy.logwarn('cannot generate a plan due to staying in a obstacle dangerous %:' + str(self.JPS_map[self.start_from]))
                # print start
                return None
            path = None
            path = self.JPS_()
            if path != None:
                # return self.get_full_path(path)
                return path
            else:
                rospy.logwarn('Unvalid Goal, No Path founded')
                return None
        else:
            rospy.logwarn('waiting for map... ')
            return None

    def JPS_(self):
        rospy.loginfo('start JPS algrithm')
        if self.JPS_map != None:
            self.field = [i for i in self.JPS_map]
            self.field[self.start_from] = self.ORIGIN
            self.field[self.end_with] = self.DESTINATION
            self.sources = dict()
            self.ADD_JUMPPOINT(self.start_from)
            while not self.Queue.empty():
                node = self.Queue.pop_task()
                try:
                    self.ADD_JUMPPOINT(self.explore_straight(node, 1, 0))
                    self.ADD_JUMPPOINT(self.explore_straight(node, -1, 0))
                    self.ADD_JUMPPOINT(self.explore_straight(node, 0, 1))
                    self.ADD_JUMPPOINT(self.explore_straight(node, 0, -1))

                    self.ADD_JUMPPOINT(self.explore_diagonal(node, 1, 1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, 1, -1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, -1, 1))
                    self.ADD_JUMPPOINT(self.explore_diagonal(node, -1, -1))
                except FoundPath:
                    rospy.loginfo('found path')
                    return self.generate_path_jump_point()
        else:
            rospy.logwarn('waiting for map')

    def ADD_JUMPPOINT(self, node):
        if node != None:
            self.Queue.add_task(node, (self.field[node] + numpy.sqrt((((node - self.end_with)/self.mapinfo.width)**2 + ((node - self.end_with)%self.mapinfo.width)**2))))

    def explore_diagonal(self, node, direction_x, direction_y):
        cur_node = copy.deepcopy(node)
        cur_cost = copy.deepcopy(self.field[node])
        while True:
            cur_node = cur_node + direction_y*self.mapinfo.width + direction_x
            cur_cost += 1.414
            if self.field[cur_node] == self.UNINITIALIZED:
                self.field[cur_node] = cur_cost
                self.sources[cur_node] = node
            elif cur_node == self.end_with:
                self.field[cur_node] = cur_cost
                self.sources[cur_node] = node
                raise FoundPath()
            else:
                return None

            if self.field[cur_node + direction_x] >= self.obstacle_thread and self.field[cur_node + direction_x + direction_y*self.mapinfo.width] < self.obstacle_thread:
                return cur_node
            else:
                self.ADD_JUMPPOINT(self.explore_straight(cur_node, direction_x, 0))
            if self.field[cur_node + direction_y*self.mapinfo.width] >= self.obstacle_thread and self.field[cur_node + direction_x + direction_y*self.mapinfo.width] < self.obstacle_thread:
                return cur_node
            else:
                self.ADD_JUMPPOINT(self.explore_straight(cur_node, 0, direction_y))

    def explore_straight(self, node, direction_x, direction_y):
        cur_node = node
        cur_cost = self.field[node]
        while True:
            cur_node = cur_node + direction_y*self.mapinfo.width + direction_x
            cur_cost += 1
            if self.field[cur_node] == self.UNINITIALIZED:
                self.field[cur_node] = cur_cost
                self.sources[cur_node] = node
            elif cur_node == self.end_with:
                self.field[cur_node] = cur_cost
                self.sources[cur_node] = node
                raise FoundPath()
            else:
                return None

            if direction_x == 0:
                if self.field[cur_node + 1] >= self.obstacle_thread and self.field[cur_node + direction_y*self.mapinfo.width + 1] < self.obstacle_thread:
                    return cur_node
                if self.field[cur_node - 1] >= self.obstacle_thread and self.field[cur_node + direction_y*self.mapinfo.width - 1] < self.obstacle_thread:
                    return cur_node
            if direction_y == 0:
                if self.field[cur_node + self.mapinfo.width] >= self.obstacle_thread and self.field[cur_node + self.mapinfo.width + direction_x] < self.obstacle_thread:
                    return cur_node
                if self.field[cur_node - self.mapinfo.width] >= self.obstacle_thread and self.field[cur_node - self.mapinfo.width + direction_x] < self.obstacle_thread:
                    return cur_node

    def generate_path_jump_point(self):
        path = []
        cur_node = self.end_with
        # print 'sources: ', self.sources
        while cur_node != self.start_from:
            pose = PoseStamped()
            pose.pose.position.x = cur_node%self.mapinfo.width*self.mapinfo.resolution + self.mapinfo.origin.position.x
            pose.pose.position.y = cur_node/self.mapinfo.width*self.mapinfo.resolution + self.mapinfo.origin.position.y
            path.append(pose)
            cur_node = self.sources[cur_node]
        if len(path) > 1:
            path.reverse()
            startpose = [PoseStamped()]
            startpose[0].pose.position.x = self.start_from%self.mapinfo.width * self.mapinfo.resolution + self.mapinfo.origin.position.x
            startpose[0].pose.position.y = self.start_from/self.mapinfo.width * self.mapinfo.resolution + self.mapinfo.origin.position.y
            path = startpose + path
            return path
        else:
            return []

    def get_full_path(self, path):
        if path == []:
            return []
        else:
            rospy.loginfo('start generate a full path')
            result = []
            cur_pose = path[0]
            for i in path[1:]:
                while abs(i.pose.position.x - cur_pose.pose.position.x) >= 0.05 or abs(i.pose.position.y - cur_pose.pose.position.y) >= 0.05:
                    x_increase = self._signum(round(i.pose.position.x - cur_pose.pose.position.x, 1))
                    y_increase = self._signum(round(i.pose.position.y - cur_pose.pose.position.y, 1))
                    cur_pose.pose.position.x += x_increase
                    cur_pose.pose.position.y += y_increase
                    result.append(copy.deepcopy(cur_pose))
            rospy.loginfo('generate a full path')
            return result

    def _signum(self, n):
        if n > 0:
            return 0.05
        elif n < 0:
            return -0.05
        else:
            return 0

class A_Star():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

class D_Star():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

class Dijkstar():
    def __init__(self, end, start, mapdata):
        self.define()

    def define(self):
        pass

