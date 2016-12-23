#!/usr/bin/env python
#coding=utf-8
""" 
导航全局路径生成

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import PlanAlgrithmsLib
import collections
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import OccupancyGrid
from threading import Lock

class ClearParams:
    def __init__(self):
        rospy.delete_param('~GoalTopic')
        rospy.delete_param('~PlanTopic')
        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~OdomTopic')



class Planner():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.GoalTopic, PointStamped, self.GoalCB)
        #rospy.Subscriber(self.OdomTopic, Pose, self.OdomCB)
        #rospy.Subscriber(self.MapTopic, OccupancyGrid, self.MapCB)
        rospy.Timer(self.period, self.PubPlanCB)
        rospy.spin()

    #def OdomCB(self, data):
        #self.odom = data.position

    def GoalCB(self, data):
        with self.locker:
            self.PlanHandle(data)

    # def MapCB(self, data):
    #     self.mapdata = data

    def PlanHandle(self, data):
        plan = Path()
        plan.header = data.header
        end = data.point
        odom = rospy.wait_for_message(self.OdomTopic, Pose)
        start = odom.position
        mapdata = rospy.wait_for_message(self.MapTopic, OccupancyGrid)
        plan.poses = PlanAlgrithmsLib.AlgrithmsLib.JPS.get_path(end, start, mapdata)
        self.plans.put(plan)


    def PubPlanCB(self, data):
        if not self.plans.empty():
            rospy.loginfo('update plan')
            self.PubPlan = self.plans.get()
        else:
            if self.PubPlan != Path():
                pub = rospy.Publisher(self.PlanTopic, Path, queue_size=1)
                pub.publish(self.PubPlan)
            else:
                rospy.loginfo('please give the frist goal')
                pass


 def define(self):
     if not rospy.has_param('~GoalTopic'):
         rospy.set_param('~GoalTopic', '/clicked_point')
     self.GoalTopic = rospy.get_param('~GoalTopic')

     if not rospy.has_param('~MapTopic'):
         rospy.set_param('~MapTopic', '/map')
     self.MapTopic = rospy.get_param('~MapTopic')

     if not rospy.has_param('~PlanTopic'):
         rospy.set_param('~PlanTopic', '/move_base/action_plan/jps')
     self.PlanTopic = rospy.get_param('~PlanTopic')

     if not rospy.has_param('~PublishFrequency'):
         rospy.set_param('~PublishFrequency', 0.01)
     self.PublishFrequency = rospy.get_param('~PublishFrequency')

     if not rospy.has_param('~OdomTopic'):
          rospy.set_param('~OdomTopic', '/robot_position_in_map')
     self.OdomTopic = rospy.get_param('~OdomTopic')

     self.period = rospy.Duration(self.PublishFrequency)
     self.odom = Pose()
     self.locker = Lock()
     self.plans = collections.deque(maxlen=1)
     self.PubPlan = Path()

if __name__=='__main__':
     rospy.init_node('Plantest')
     try:
         rospy.loginfo( "initialization system")
         Planner()
         ClearParams()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")

