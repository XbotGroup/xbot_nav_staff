#!/usr/bin/env python
# coding=utf-8

"""
博物馆走走停停

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from nav_msgs.msg import Path
import collections
from geometry_msgs.msg import PointStamped
from PlanAlgrithmsLib import AlgrithmsLib
from PlanAlgrithmsLib import PathLib
import getpass
from nav_msgs.srv import GetMap
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

Finish = False
path_q = collections.deque()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PlanTopic_view')
        rospy.delete_param('~PathStorePath')
        rospy.delete_param('~times')
        rospy.delete_param('~StopRun_RUN_Topic')
        rospy.delete_param('~Speak_Done_Topic')

class StopRun():
    def __init__(self):
        rospy.sleep(1.0)
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        rospy.Timer(self.period, self.PubPlan_viewCB)
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

        usr_name = getpass.getuser()
        file = "/home/%s/"%usr_name

        if not rospy.has_param('~PathStorePath'):
            rospy.set_param('~PathStorePath', 'path.json')
        self.file = file + rospy.get_param('~PathStorePath')

        if not rospy.has_param('~times'):
            rospy.set_param('~times', 2)
        self.times = rospy.get_param('~times')

        if not rospy.has_param('~StopRun_RUN_Topic'):
            rospy.set_param('~StopRun_RUN_Topic', '/StopRun_run')
        self.StopRun_RUN_Topic = rospy.get_param('~StopRun_RUN_Topic')

        if not rospy.has_param('~Speak_Done_Topic'):
            rospy.set_param('~Speak_Done_Topic', '/speak_done')
        self.Speak_Done_Topic = rospy.get_param('~Speak_Done_Topic')

        self.period = rospy.Duration(0.01)
        self.JPS = AlgrithmsLib.JPS()
        self.reset_data()
        self.StopRun_ = rospy.Publisher(self.StopRun_RUN_Topic, Bool, queue_size=1)
        # self.varify_connection()

    # def varify_connection(self):
    #     rospy.logwarn('varify_connection with topic: '+ str(self.Speak_Done_Topic))
    #     while True:
    #         try:
    #             if not rospy.wait_for_message(self.Speak_Done_Topic, Bool, timeout= 0.2).data:
    #                 rospy.logwarn('varify_connection done')
    #                 self.StopRun_.publish(True)
    #                 break
    #         except:
    #             pass


    def reset_data(self):
        self.start = None
        self.seq = 0
        self.pub_seq = 0
        self.path = collections.deque()
        self.reviewpath = []
        self.path_pub = Path()
        # self.init_stack()
        self.init = True

    def init_stack(self):
        rospy.logwarn('StopRun: make sure your robot stoped!!')
        rospy.loginfo('StopRun: waiting for odom')
        self.start = rospy.wait_for_message(self.OdomTopic, PoseStamped).pose.position
        rospy.loginfo('StopRun: get odom')

    def PlanCB(self, event):
        global Finish
        try:
            if not Finish:
                Finish = rospy.wait_for_message(self.Speak_Done_Topic, Bool).data
                if len(self.path) == 0:
                    Finish = False
                    rospy.logwarn('StopRun: No path generated not start yet')
                else:
                    rospy.logwarn('StopRun: Finish: ' + str(Finish))
                if Finish:
                    paths = list(self.path)
                    paths *= 2
                    self.path.clear()
                    [self.path.append(i) for i in paths]
                self.actionpath = self.path.popleft()
        except:
            pass
        if Finish:
            if self.pub_seq <= 5:
                self.pub_seq = self.publish_data(self.PlanTopic, self.actionpath, self.pub_seq)
            cur_position = rospy.wait_for_message('/robot_position_in_map', PoseStamped).pose.position
            if self.arrive_check(cur_position, self.actionpath[-1].pose.position):
                for i in range(2):
                    self.StopRun_.publish(True)
                if len(self.path) == 0:
                    rospy.signal_shutdown('restart')
                else:
                    if rospy.wait_for_message(self.Speak_Done_Topic, Bool).data:
                        self.pub_seq = 0
                        self.actionpath = self.path.popleft()
                    else:
                        rospy.logwarn('StopRun: wait for speak done')

    def arrive_check(self, cur, goal):
        if abs(cur.x - goal.x) <= 0.01 and abs(cur.y - goal.y) <= 0.01:
            return True
        else:
            return False

    def GoalCB(self, goal):
        global Finish
        if not Finish:
            if self.init:
                self.init_stack()
                self.init = False
            if self.start != None:
                path = self.Generate_path(self.start, goal.point)
                if path != []:
                    self.start = goal.point
                    global path_q
                    path_q.append(path)
                else:
                    rospy.logwarn('StopRun: goal not valide')
            else:
                rospy.logwarn('StopRun: wait fot odom please click again')
                self.init_stack()

    def PubPlan_viewCB(self, event):
        global path_q
        if len(path_q) > 0:
            path = path_q.pop()
            self.reviewpath += path
            self.path.append(path)
        if not Finish:
            self.seq = self.publish_data(self.PlanTopic_view, self.reviewpath, self.seq)

    def publish_data(self, Topic, data, seq):
        PubPlan = Path()
        PubPlan.header.seq = seq
        seq += 1
        PubPlan.header.stamp = rospy.Time.now()
        PubPlan.header.frame_id = 'map'
        PubPlan.poses = data
        if data != []:
            pub = rospy.Publisher(Topic, Path, queue_size=1)
            pub.publish(PubPlan)
        return seq

    def Generate_path(self, start, end):
        rospy.wait_for_service('/JPS_map_init')
        service = rospy.ServiceProxy('/JPS_map_init', GetMap)
        map_resp = service()
        map_message = map_resp.map
        self.JPS.get_map(map_message)
        if start != None and end != None:
            path = self.JPS.get_path(end, start)
            return self.JPS.get_full_path(path)
        else:
            return []

