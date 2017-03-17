#!/usr/bin/env python
#coding=utf-8
""" 
导航全局路径生成

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
from PlanAlgrithmsLib import AlgrithmsLib
from PlanAlgrithmsLib import maplib
import collections
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from threading import Lock
import time
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from nav_msgs.srv import GetMap

timer = time.time()
NO_patch = True

class ClearParams:
    def __init__(self):
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~MapTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~visual_debug')
        rospy.delete_param('~detect_scale')


class Planner():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        # rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Timer(rospy.Duration(0.05), self.ChrashChecker)
        rospy.spin()

    # def OdomCB(self, odom_messge):
    #     self.odom = odom_messge

    def GoalCB(self, data):
        self.MakePlan(data)

    def MakePlan(self, data):
        rospy.loginfo('get a new goal')
        # map_message = rospy.wait_for_message(self.MapTopic, OccupancyGrid)
        rospy.wait_for_service('/JPS_map_init')
        service = rospy.ServiceProxy('/JPS_map_init', GetMap)
        map_resp = service()
        map_message = map_resp.map
        self.JPS.get_map(map_message)
        odom = rospy.wait_for_message(self.OdomTopic, PoseStamped)
        if odom != None:
            self.goal = data
            end = self.goal.point
            start = odom.pose.position
            plan = self.Generate_plan(end, start)
            if plan.poses != None:
                self.PubPlanCB(plan)
        else:
            rospy.logwarn('waiting for odom...')

    def Generate_plan(self, end, start):
        plan = Path()
        plan.header.seq = self.seq
        self.seq += 1
        plan.header.frame_id = 'map'
        path = self.JPS.get_path(end, start)
        if path != None:
            self.visualise([i.pose.position for i in path])
            plan.poses = self.JPS.get_full_path(path)
        return plan

    def ChrashChecker(self, event):
        try:
            map_message = rospy.wait_for_message(self.MapTopic, OccupancyGrid)
            if self.Chrash(map_message):
                self.JPS.get_map(map_message)
        except:
            pass

    def Chrash(self, map_message):
        blocked = maplib.get_effective_point(map_message)
        if self.Pose_Checker(self.PubPlan.poses, blocked):
            if self.visual_debug:
                self.block_visual_testing(blocked)
            rospy.logwarn('detect obstacles and rebuilding plan...')
            self.MakePlan(self.goal)
            return True
        else:
            return False

    def Pose_Checker(self, poses, blocked):
        if poses != None and len(poses) > 1:
            if len(poses) >= self.detect_scale:
                data_set = [i.pose.position for i in poses[:self.detect_scale]]
            else:
                data_set = [i.pose.position for i in poses]
            for i in data_set:
                for j in blocked:
                    if abs(i.x-j.x) <= 0.01 and abs(i.y-j.y) <= 0.01:
                        return True
            return False
        else:
            global NO_patch
            if NO_patch:
                rospy.logwarn('No valid path')
                NO_patch = False
            return False

    def block_visual_testing(self, data):
        color = ColorRGBA()
        scale = Point()
        scale.x = 0.05
        scale.y = 0.05
        color.r = 0.0
        color.g = 1.0
        color.b = 0.0
        color.a = 1.0
        result = maplib.visual_test(data, Marker.POINTS, color, scale)
        pub = rospy.Publisher('/staticarea_marker', Marker, queue_size=1)
        pub.publish(result)


    def visualise(self, nodes):
        if self.visual_debug:
            rospy.loginfo('visualising JPS nodes')
            color = ColorRGBA()
            scale = Point()
            scale.x = 0.05
            scale.y = 0.05
            color.r = 0.0
            color.g = 1.0
            color.a = 1.0
            result = maplib.visual_test(nodes, Marker.POINTS, color, scale, 0)
            pub = rospy.Publisher('/JPS_nodes', Marker, queue_size=1)
            pub.publish(result)

    def PubPlanCB(self, plan):
        rospy.loginfo('get new plan....')
        self.PubPlan = Path()
        self.PubPlan = plan
        self.PubPlan.header.stamp = rospy.Time.now()
        self.PubPlan.header.seq = self.seq
        self.seq += 1
        pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
        pub.publish(self.PubPlan)
        rospy.loginfo('publshing a new plan')

    def define(self):
        if not rospy.has_param('~GoalTopic'):
            rospy.set_param('~GoalTopic', '/clicked_point')
        self.GoalTopic = rospy.get_param('~GoalTopic')

        if not rospy.has_param('~MapTopic'):
            rospy.set_param('~MapTopic', '/cost_plan_map')
        self.MapTopic = rospy.get_param('~MapTopic')

        if not rospy.has_param('~PlanTopic'):
            rospy.set_param('~PlanTopic', '/move_base/action_plan/jps')
        self.PlanTopic = rospy.get_param('~PlanTopic')

        if not rospy.has_param('~OdomTopic'):
             rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~visual_debug'):
             rospy.set_param('~visual_debug', True)
        self.visual_debug = rospy.get_param('~visual_debug')

        if not rospy.has_param('~detect_scale'):
             rospy.set_param('~detect_scale', 30)
        self.detect_scale = rospy.get_param('~detect_scale')

        self.JPS = AlgrithmsLib.JPS()
        # self.odom = None
        self.seq = 0

if __name__=='__main__':
     rospy.init_node('Planner')
     try:
         rospy.loginfo( "initialization system")
         Planner()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")