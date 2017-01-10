#!/usr/bin/env python
# coding=utf-8

"""
规划固定路径

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import collections
from geometry_msgs.msg import PointStamped
from PlanAlgrithmsLib import AlgrithmsLib
import copy
from PlanAlgrithmsLib import PathLib
import os
import getpass

Finish = False
Save = False
use_exit_path = False

class ClearParams:
    def __init__(self):
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PlanTopic_view')
        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~PathStorePath')


class fixed():
    def __init__(self):
        self.define()
        # rospy.Subscriber(self.PlanTopic, Path, self.PlanCB)
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.Timer(self.period, self.FinshCB)
        rospy.Timer(self.period, self.PlanCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')
        self.GoalTopic = rospy.get_param('~GoalTopic')

        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan/fixed')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~PlanTopic_view'):
            rospy.set_param('~PlanTopic_view', '/move_base/action_plan/view_fixed')
        self.PlanTopic_view = rospy.get_param('~PlanTopic_view')

        if not rospy.has_param('~PublishFrequency'):
            rospy.set_param('~PublishFrequency', 0.01)
        PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~PathStorefile'):
            usr_name = getpass.getuser()
            file = "/home/%s/path.json" % usr_name
            rospy.set_param('~PathStorePath', file)
        self.file = rospy.get_param('~PathStorePath')


        self.period = rospy.Duration(PublishFrequency)
        self.JPS = AlgrithmsLib.JPS()

        self.reset_data()
        self.detect_store_path()

    def reset_data(self):
        self.start = None
        self.store = None
        self.seq = 0
        self.path = []
        self.save_data = []
        self.path_pub = Path()
        self.path_q = collections.deque(maxlen=1)
        self.init_stack()

    def init_stack(self):
        self.start = rospy.wait_for_message(self.OdomTopic, PoseStamped).pose.position
        if self.start != None:
            self.JPS_Points = collections.deque(maxlen=1)
            if self.start not in self.JPS_Points:
                self.JPS_Points.append(self.start)
                rospy.loginfo('initial data set')

    def detect_store_path(self):
        if os.path.isfile(self.file):
            res = raw_input('do you want to use store data? press ' + ' \033[1;31;31m y/Y \033[0m' + ' to load path. or press other key to continue\n')
            if res.lower() == 'y':
                data = PathLib.read_path(self.file)
                path = PathLib.get_store_path(data)
                self.path = path
                global use_exit_path
                use_exit_path = True
                self.publish_data(self.PlanTopic, self.path)

    def PlanCB(self, event):
        global use_exit_path
        if use_exit_path:
            if self.seq <= 100:
                self.publish_data(self.PlanTopic, self.path)

    def GoalCB(self, goal):
        global Finish
        global use_exit_path
        if not use_exit_path:
            if not Finish:
                if self.start != None:
                    point = goal.point
                    if point not in self.JPS_Points:
                        rospy.loginfo('obtain a point')
                        self.JPS_Points.append(point)
                        rospy.loginfo('points in store')
                        path = self.Generate_path()
                        if path != None:
                            self.store = self.start
                            self.path_q.append(path)
                        else:
                            self.JPS_Points.append(self.store)
                            path = self.Generate_path()
                else:
                    rospy.logwarn('wait fot odom')
                    self.init_stack()
            # else:
            #     self.reset_data()

            rospy.loginfo('\npress' + ' \033[1;31;31m e/E \033[0m' + ' to end process\n' + 'press' + ' \033[1;31;31m r/R \033[0m ' + 'to restart\n' + 'press' + ' \033[1;31;31m s/S \033[0m' + ' to save path\n')
        else:
            res = raw_input('do you want to make a new path? press ' + ' \033[1;31;31m y/Y \033[0m' + ' to make a new path\n')
            if res.lower() == 'y':
                use_exit_path = False
                self.reset_data()

    def PubPlanCB(self, event):
        global use_exit_path
        if not use_exit_path:
            if len(self.path_q) > 0:
                self.path += self.path_q.pop()
                self.save_data = self.path
                self.seq = 0
                if not Finish:
                    self.publish_data(self.PlanTopic_view, self.path)
            else:
                if Finish:
                    if self.seq <= 100:
                        self.publish_data(self.PlanTopic, self.path)

                else:
                    self.publish_data(self.PlanTopic_view, self.path)


    def publish_data(self, Topic, data):
        PubPlan = Path()
        PubPlan.header.seq = self.seq
        self.seq += 1
        PubPlan.header.stamp = rospy.Time.now()
        PubPlan.header.frame_id = 'map'
        PubPlan.poses = data
        if data != []:
            pub = rospy.Publisher(Topic, Path, queue_size=1)
            pub.publish(PubPlan)

    def Generate_path(self):
        if len(self.JPS_Points) > 0:
            end = self.JPS_Points.pop()
            start = copy.deepcopy(self.start)
            self.store = start
            self.start = end
            return self.JPS.get_path(end, start)
        else:
            return None

    def FinshCB(self, event):
        global Finish
        global Save
        global use_exit_path
        if not use_exit_path:
            if not Finish:
                res = raw_input('press' + ' \033[1;31;31m e/E \033[0m' + ' to end process\n' + 'press' + ' \033[1;31;31m r/R \033[0m ' + 'to restart\n' + 'press' + ' \033[1;31;31m s/S \033[0m' + ' to save path\n')
                if res.lower() == 'e':
                    Finish = True
                    self.seq = 0
                    rospy.loginfo('end progress')
                if res.lower() == 'r':
                    Finish = False
                    rospy.loginfo('restart progress')
                if res.lower() == 's':
                    rospy.loginfo('ending progress and saving data...')
                    Save = True
                    Finish = True
            else:
                if not Save:
                    rospy.loginfo('\npress' + ' \033[1;31;31m r/R \033[0m ' + 'to restart\n' + 'press' + ' \033[1;31;31m s/S \033[0m' + ' to save path\n')
                else:
                    rospy.loginfo('\npress' + ' \033[1;31;31m r/R \033[0m ' + 'to restart\n')
                res = raw_input('')
                if res.lower() == 'r':
                    Finish = False
                    Save = False
                    rospy.loginfo('restart progress')
                    self.reset_data()

                if not Save:
                    if res.lower() == 's':
                        rospy.loginfo('ending progress and saving data...')
                        Save = True
                        Finish = True
            if Save:
                PathLib.save_path(self.save_data, self.file)
                rospy.loginfo('data saved...')
                self.reset_data()

if __name__=='__main__':
     rospy.init_node('fixed_plan_maker')
     try:
         rospy.loginfo( "initialization system")
         fixed()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")
