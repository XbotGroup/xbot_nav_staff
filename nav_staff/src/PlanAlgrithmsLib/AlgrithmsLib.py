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
import heaqp
import itertools
from geometry_msgs.msg import PoseStamped

class JPS():
    ##########################################################################################################
    # JUMP-POINT SEARCH                                                                                      #
    #  Described: https://harablog.wordpress.com/2011/09/07/jump-point-search/, and thanks Christopher Chu   #
    #                                                                                                        #
    # Using this function:                                                                                   #
    #   - use generate_field(...) to create an 2d array denoting which cells are walkable                    #
    #   - use jps(...) to get paths.                                                                         #
    #   - use get_full_path(...) on the result from jps to get every cell in the path.                       #
    #                                                                                                        #
    # Note: this implementation allows diagonal movement and "corner cutting"                                #
    #                                                                                                        #
    ##########################################################################################################

    def __init__(self, end, start, mapdata):
        self.define(mapdata)

    def define(self, map_message):
        self.expanded_field = [[False for i in range(map_message.info.width)] for j in range(map_message.info.height)]
        self.visited_field = [[False for i in range(map_message.info.width)] for j in range(map_message.info.height)]

        if not rospy.has_param('~obstacle_thread'):
            rospy.set_param('~obstacle_thread', 80)
        self.obstacle_thread = rospy.get_param('~obstacle_thread')

        self.UNINITIALIZED = -1
        self.OBSTACLE = 100
        self.ORIGIN = 0
        self.DESTINATION = -2

    def get_path(self, end, start, map_message):
        rospy.loginfo('starting gernerating plan')
        JPS_map = self.generate_map(map_message)
        start_from = (int(start.x / map_message.info.resolution), int(start.y / map_message.info.resolution))
        end_with = (int(end.x / map_message.info.resolution), int(end.y / map_message.info.resolution))
        path = self.JPS_(end_with, start_from, JPS_map)

    def generate_map(self, map_message):
        field = numpy.array(map_message)
        field = field.reshape(map_message.info.height, map_message.info.width)
        field = [[ self.UNINITIALIZED if field[i][j] < self.obstacle_thread else self.OBSTACLE for j in ] for i in range(map_message.info.height)]
        return field

    def JPS_(self, end, start, map):
        if map[end[0]][end[1]] == self.OBSTACLE:
            rospy.loginfo('goal is not walkable')
            pass
        if map[start[0]][start[1]] == self.OBSTACLE:
            rospy.loginfo('cannot walk due to staying in a obstacle')
        field = [[j for j in i] for i in map] # this takes less time than deep copying.
        sources = [[(None, None) for i in field[0]] for j in field]  # the jump-point predecessor to each point.
        field[start[0]][start[1]] = self.ORIGIN
        field[end[0]][end[1]] = self.DESTINATION
        Queue = PriorityQueue()
        self.ADD_JUMPPOINT((start, Queue, field, end))
        while not Queue.empty():
            node = Queue.pop_task()
            try:
                self.ADD_JUMPPOINT(self.explore_diagonal(node, 1, 0, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_diagonal(node, -1, 0, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_diagonal(node, 0, 1, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_diagonal(node, 0, -1, field, sources, end))

                self.ADD_JUMPPOINT(self.explore_cardinal(node, 1, 1, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_cardinal(node, 1, -1, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_cardinal(node, -1, 1, field, sources, end))
                self.ADD_JUMPPOINT(self.explore_cardinal(node, -1, -1, field, sources, end))
            except FoundPath:
                return self.generate_path_jump_point(sources, start, end)


    def ADD_JUMPPOINT(self, node, Queue, field, end):
        if node is not None:
            Queue.add_task(node, field[node[0]][node[1]] + max(node[0] - end[0]), abs(node[1] - end[1]))

    def explore_diagonal(self, start, direction_x, direction_y, field, sources, end):
        cur_x = start[0]
        cur_y = start[1]
        cur_cost = field[start[0]][start[1]]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1
            if field[cur_x][cur_y] == self.UNINITIALIZED
                field[cur_x][cur_y] = cur_cost
                sources[cur_x][cur_y] = start[0], start[1]
            elif cur_x == end[0] and cur_y == end[1]:
                field[cur_x][cur_y] = cur_cost
                sources[cur_x][cur_y] = start[0], start[1]
                raise FoundPath
            else:
                return None
            #if a jump point is found
            if field[cur_x + direction_x][cur_y] == self.OBSTACLE and field[cur_x + direction_x][cur_y + direction_y] != self.OBSTACLE:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_cardinal((cur_x, cur_y), directionX, 0, field, sources, end))
            if field[cur_x][cur_y + direction_y] == self.OBSTACLE and field[cur_x + direction_x][cur_y + direction_y] != self.OBSTACLE:
                return (cur_x, cur_y)
            else:
                self.ADD_JUMPPOINT(self.explore_cardinal((cur_x, cur_y), 0, direction_y, field, sources, end))

    def explore_cardinal(self, start, direction_x, direction_y, field, sources, end):
        cur_x = start[0]
        cur_y = start[1]
        cur_cost = field[start[0]][start[1]]
        while True:
            cur_x += direction_x
            cur_y += direction_y
            cur_cost += 1
            if field[cur_x][cur_y] == self.UNINITIALIZED:
                field[cur_x][cur_y] = cur_cost
                sources[cur_x][cur_y] = start
            elif cur_x == end[0] and cur_y == end[1]:
                field[cur_x][cur_y] = cur_cost
                sources[cur_x][cur_y] = start
                raise FoundPath
            else:
                return None

            if direction_x == 0:
                if field[cur_x + 1][cur_y] == self.OBSTACLE and field[cur_x + 1][cur_y + direction_y] != self.OBSTACLE:
                    return (cur_x, cur_y)
                if field[cur_x - 1][cur_y] == self.OBSTACLE and field[cur_x - 1][cur_y + direction_y] != self.OBSTACLE:
                    return (cur_x, cur_y)
            elif direction_y == 0 :
                if field[cur_x][cur_y + 1] == self.OBSTACLE and field[cur_x + direction_x][cur_y + 1] != self.OBSTACLE:
                    return (cur_x, cur_y)
                if field[cur_x][cur_y - 1] == self.OBSTACLE and field[cur_x + direction_x][cur_y - 1] != self.OBSTACLE:
                    return (cur_x, cur_y)

    def generate_path_jump_point(self, sources, start, end):
        """
        Reconstruct the jump_point consisted path from the source information as given by jps(...).

        Parameters
        sources          - a 2d array of the predecessor to each node
        start            - the x, y coordinates of the starting position
        end              - the x, y coordinates of the destination

        Return
        posestamped list
        """
        path = list()
        cur_x = end[0]
        cur_y = end[1]
        while cur_x != start[0] or cur_y != start[1]:
            pose = PoseStamped()
            pose.pose.position.x = cur_x
            pose.pose.position.y = cur_y
            path.append(pose)
            cur_x, cur_y = sources[cur_x][cur_y]
        path.reverse()
        startpose = PoseStamped()
        startpose.pose.position.x = start[0]
        startpose.pose.position.y = start[1]
        path = startpose + path
        return path

    def get_path(self, path):
        """
        Generates the full path from a list of jump points. Assumes that you moved in only one direction between
        jump points.

        Return
        posestamped list
        """
        if path == []:
            return []
        else:
            cur_pose = path[0]
            result = [path[0]]
            for i in range(len(path) - 1):
                while cur_pose.pose.position.x !=path[i + 1].pose.position.x or cur_pose.pose.position.y != path[i +1].pose.position.y:
                    cur_pose.pose.position.x += self._signum(path[i + 1].pose.position.x - path[i].pose.position.x)
                    cur_pose.pose.position.y += self._signum(path[i + 1].pose.position.y - path[i].pose.position.y)
                    result.append(cur_pose)
            return result

    def _signum(self, n):
        if n > 0:
            return 1
        elif n < 0:
            return -1
        else:
            return 0


class PriorityQueue():

    def __init__(self):
        self.pq = []  # list of entries arranged in a heap
        self.counter = itertools.count()  # unique sequence count

    def add_task(self, task, priority=0):
        'Add a new task'
        count = next(self.counter)
        entry = [priority, count, task]
        heapq.heappush(self.pq, entry)

    def pop_task(self):
        'Remove and return the lowest priority task. Raise KeyError if empty.'
        while self.pq:
            priority, count, task = heapq.heappop(self.pq)
            return task
        raise KeyError('pop from an empty priority queue')

    def empty(self):
        return len(self.pq) == 0

class FoundPath(Exception):
    pass

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

