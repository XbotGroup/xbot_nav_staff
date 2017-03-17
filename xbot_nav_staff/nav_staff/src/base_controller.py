#!/usr/bin/env python
# coding=utf-8
"""
底盘移动控制软件

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import numpy
from PlanAlgrithmsLib import CVlib
from PlanAlgrithmsLib import maplib
import collections
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from threading import Lock
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String

Tasks = list()
cmd_queue = None
switcher = False

class ClearParams:
    def __init__(self):
        rospy.delete_param('~SwitchModleTopic')
        rospy.delete_param('~PlanTopicFixed')
        rospy.delete_param('~PlanTopicOnce')

        rospy.delete_param('~OdomTopic')
        rospy.delete_param('~MotionTopice')

        rospy.delete_param('~PathAcc')
        rospy.delete_param('~MaxLinearSP')
        rospy.delete_param('~MinLinearSP')
        rospy.delete_param('~AngularSP')

        rospy.delete_param('~AngularBias')
        rospy.delete_param('~AngularFree')
        rospy.delete_param('~PublishFrequency')
        rospy.delete_param('~GoalTolerant')

class BaseController:
    def __init__(self):
        self.define()
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Subscriber(self.PlanTopicFixed, Path, self.PlanFixedCB)
        rospy.Subscriber(self.PlanTopicOnce, Path, self.PlanOnceCB)
        rospy.Subscriber(self.SwitchModleTopic, String, self.SwitchCB)
        rospy.Timer(self.period, self.PubcmdCB)
        rospy.spin()

    def define(self):
        # parameters
        if not rospy.has_param('~SwitchModleTopic'):
            rospy.set_param('~SwitchModleTopic', '/move_base/switch')
        self.SwitchModleTopic = rospy.get_param('~SwitchModleTopic')

        if not rospy.has_param('~PlanTopicFixed'):
            rospy.set_param('~PlanTopicFixed', '/move_base/action_plan/fixed')
        self.PlanTopicFixed = rospy.get_param('~PlanTopicFixed')

        if not rospy.has_param('~PlanTopicOnce'):
            rospy.set_param('~PlanTopicOnce', '/move_base/action_plan/once')
        self.PlanTopicOnce = rospy.get_param('~PlanTopicOnce')

        if not rospy.has_param('~OdomTopic'):
            rospy.set_param('~OdomTopic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~OdomTopic')

        if not rospy.has_param('~MotionTopice'):
            # cmd_vel_mux/input/navi #/navigation_velocity_smoother/raw_cmd_vel
            rospy.set_param('~MotionTopice', 'cmd_vel_mux/input/smoother')
        self.MotionTopice = rospy.get_param('~MotionTopice')

        # how accuracy the robot will attemped to move to next path goal
        if not rospy.has_param('~PathAcc'):
            rospy.set_param('~PathAcc', 0.5)
        self.PathAcc = rospy.get_param('~PathAcc')

        if not rospy.has_param('~MaxLinearSP'):
            rospy.set_param('~MaxLinearSP', 0.5)
        self.MaxLinearSP = rospy.get_param('~MaxLinearSP')

        if not rospy.has_param('~MinLinearSP'):
         rospy.set_param('~MinLinearSP', 0.1)
        self.MinLinearSP = rospy.get_param('~MinLinearSP')

        if not rospy.has_param('~AngularSP'):
         rospy.set_param('~AngularSP', 0.3)
        self.AngularSP = rospy.get_param('~AngularSP')

        if not rospy.has_param('~AngularBias'):
            rospy.set_param('~AngularBias', 0.3)
        self.AngularBias = rospy.get_param('~AngularBias')

        if not rospy.has_param('~AngularFree'):
            rospy.set_param('~AngularFree', 0.1745)
        self.AngularFree = rospy.get_param('~AngularFree')

        if not rospy.has_param('~PublishFrequency'):
         rospy.set_param('~PublishFrequency', 0.01) #100hz
        self.PublishFrequency = rospy.get_param('~PublishFrequency')

        if not rospy.has_param('~GoalTolerant'):
         rospy.set_param('~GoalTolerant', 0.01)
        self.GoalTolerant = rospy.get_param('~GoalTolerant')

        if not rospy.has_param('~times'):
            rospy.set_param('~times', 2)
        self.times = rospy.get_param('~times')

        self.path = []

        self.period = rospy.Duration(self.PublishFrequency)

        self.locker = Lock()

        self.cmd_vel = Twist()

    def SwitchCB(self, signal):
        global switcher
        if signal.data == 'FixedModule':
            rospy.logwarn('Switch to FixedModule')
            switcher = False
        elif signal.data == 'OnePathModule':
            rospy.logwarn('Switch to OnePathModule')
            switcher = True

    def OdomCB(self, odom):
        global Tasks
        cur_pose = odom.pose
        if Tasks != []:
            cur_goal = Tasks[0]
            if abs(round(cur_pose.position.x - cur_goal.x, 2)) <= self.GoalTolerant and abs(round(cur_pose.position.y - cur_goal.y, 2)) <= self.GoalTolerant:
                Tasks.remove(cur_goal)
                rospy.loginfo('arrive goal')
                self.cmd_vel = Twist()
            else:
                # rospy.loginfo('go to goal...')
                self.count_cmds(cur_pose, cur_goal)

    def count_cmds(self, cur_pose, cur_goal):
        Diff_x = round(cur_goal.x - cur_pose.position.x, 2)
        Diff_y = round(cur_goal.y - cur_pose.position.y, 2)
        cur_angle = CVlib.GetAngle(cur_pose.orientation)
        self.Vector(Diff_x, Diff_y, cur_angle)

    def Vector(self, Diff_x, Diff_y, cur_angle):
        goal_linear = numpy.sqrt(Diff_x**2 + Diff_y**2)
        cmd_vector = Twist()
        # anglar
        if Diff_x > 0 and Diff_y > 0:
            goal_angle = numpy.arctan(Diff_y/Diff_x)
        elif Diff_x < 0 and Diff_y >0:
            goal_angle = numpy.pi + numpy.arctan(Diff_y/Diff_x)
        elif Diff_x < 0 and Diff_y < 0:
            goal_angle = -numpy.pi + numpy.arctan(Diff_y/Diff_x)
        elif Diff_x > 0 and Diff_y < 0:
            goal_angle = numpy.arctan(Diff_y/Diff_x)
        elif Diff_x == 0 and Diff_y != 0:
            if Diff_y > 0:
                goal_angle = numpy.pi/2.0
            elif Diff_y <0:
                goal_angle = -numpy.pi/2.0
            else:
                rospy.logerr('error type 1')
        elif Diff_y == 0 and Diff_x != 0:
            if Diff_x > 0:
                goal_angle = 0.0
            elif Diff_x < 0:
                goal_angle = -numpy.pi
            else:
                rospy.logerr('error type 2')
        elif Diff_y == 0 and Diff_x == 0:
            cmd_vector.linear.x = 0.0
            cmd_vector.angular.z = 0.0
            rospy.logwarn('Diff_x Diff_y ==0')
        else:
            rospy.logerr('unkown ')
        angle_cmd = round(goal_angle - cur_angle, 3)
        if angle_cmd > numpy.pi:
            angle_cmd = -numpy.pi*2 + angle_cmd
        if angle_cmd < -numpy.pi:
            angle_cmd = numpy.pi * 2 + angle_cmd
        cmd_vector.angular.z = round(angle_cmd, 3)
        cmd_vector.linear.x = round(goal_linear, 3)
        if 0.0 < cmd_vector.angular.z < 0.03:
            cmd_vector.angular.z = 0.0
        global cmd_queue
        cmd_queue = cmd_vector

    def AngularDrift(self, Diff_x, Diff_y):

        x_drift = Diff_x
        y_drift = Diff_y
        angular_drift = numpy.arcsin(y_drift / numpy.sqrt(x_drift ** 2 + y_drift ** 2))

        if x_drift > 0 and y_drift < 0:
            angular_drift = angular_drift

        if x_drift > 0 and y_drift > 0:
            angular_drift = angular_drift

        if x_drift < 0 and y_drift < 0:
            angular_drift = -angular_drift - numpy.pi

        if x_drift < 0 and y_drift > 0:
            angular_drift = numpy.pi - angular_drift

        return angular_drift

    def GoalOrientation(self, theta):
        orientation = Quaternion()

        if -numpy.pi < theta < -numpy.pi * 2.0 / 3.0:
            orientation.z = -numpy.sin(theta / 2.0)
            orientation.w = -numpy.cos(theta / 2.0)

        else:
            orientation.z = numpy.sin(theta / 2.0)
            orientation.w = numpy.cos(theta / 2.0)

        return orientation

    def PlanFixedCB(self, PlanPath):
        global switcher
        if not switcher:
            self.path = []
            self.path = PlanPath.poses
            global Tasks
            Tasks = list()
            segment = [i.pose.position for i in self.path]
            if len(segment) >= 2:
                Tasks = self.linear_analyse(segment)
                Tasks = Tasks * self.times

    def PlanOnceCB(self, PlanPath):
        global switcher
        if switcher:
            self.path = []
            self.path = PlanPath.poses
            global Tasks
            Tasks = list()
            segment = [i.pose.position for i in self.path]
            if len(segment) >= 2:
                Tasks = self.linear_analyse(segment)

    def PubcmdCB(self, data):
        global cmd_queue
        cmd = Twist()
        if cmd_queue != None:
            self.cmd_vel = cmd_queue
            cmd_queue = None
        cmd_pub = rospy.Publisher(self.MotionTopice, Twist, queue_size=1)
        if self.cmd_vel != Twist():
            if abs(self.cmd_vel.angular.z) > self.AngularBias:
                if abs(self.cmd_vel.angular.z) < numpy.pi:
                    if self.cmd_vel.angular.z > 0:
                        cmd.angular.z = self.AngularSP
                    else:
                        cmd.angular.z = -self.AngularSP
                else:
                    if self.cmd_vel.angular.z > numpy.pi:
                        cmd.angular.z = -self.AngularSP
                    else:
                        cmd.angular.z = self.AngularSP
                cmd_pub.publish(cmd)
            else:
                # print '<AngularBias'
                cmd.angular.z = self.cmd_vel.angular.z
                if round(abs(self.cmd_vel.angular.z), 2) != 0 or round(self.cmd_vel.linear.x, 2) != 0:
                    if abs(self.cmd_vel.angular.z) <= self.AngularFree:
                        if self.cmd_vel.linear.x >= self.PathAcc:
                            cmd.linear.x = self.MaxLinearSP
                        else:
                            if self.cmd_vel.linear.x > self.MinLinearSP:
                                cmd.linear.x = self.MinLinearSP
                            else:
                                cmd.linear.x = self.cmd_vel.linear.x
                    else:
                        if self.cmd_vel.linear.x >= self.PathAcc:
                            cmd.linear.x = self.MinLinearSP
                        else:
                            if self.cmd_vel.linear.x >= self.GoalTolerant:
                                cmd.linear.x = self.cmd_vel.linear.x
                            else:
                                cmd.linear.x = 0
                else:
                    rospy.logerr('both linear and angular input is zero!')
                cmd_pub.publish(cmd)

    def linear_analyse(self, points):
        nodes = CVlib.Linear_analyse(points)
        return nodes

    # def visualise(self, nodes):
    #     if self.visual_test:
    #         color = ColorRGBA()
    #         scale = Point()
    #         scale.x = 0.05
    #         scale.y = 0.05
    #         color.r = 0.0
    #         color.g = 0.0
    #         color.b = 1.0
    #         color.a = 1.0
    #         result = maplib.visual_test(nodes, Marker.POINTS, color, scale, 1)
    #         pub = rospy.Publisher('/base_controller_key_node', Marker, queue_size=1)
    #         pub.publish(result)
