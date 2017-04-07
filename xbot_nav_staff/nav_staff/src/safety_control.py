#!/usr/bin/env python
# coding=utf-8
"""
安全减速方案

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from xbot_msgs.msg import DockInfraRed
from geometry_msgs.msg import Twist
from PlanAlgrithmsLib import CMDLib

class ClearParams:
    def __init__(self):
        rospy.delete_param('~InfraTopic')
        rospy.delete_param('~NaviTopic')
        rospy.delete_param('~TeleTopic')
        rospy.delete_param('~accsp')


class Safty_Control():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.InfraTopic, DockInfraRed, self.SaftyCB)
        rospy.Subscriber(self.NaviTopic, Twist, self.NaviCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~InfraTopic'):
            rospy.set_param('~InfraTopic', '/sensors/dock_ir')
        self.InfraTopic = rospy.get_param('~InfraTopic')

        if not rospy.has_param('~NaviTopic'):
            rospy.set_param('~NaviTopic', '/cmd_vel_mux/input/navi')
        self.NaviTopic = rospy.get_param('~NaviTopic')

        if not rospy.has_param('~TeleTopic'):
            rospy.set_param('~TeleTopic', '/cmd_vel_mux/input/teleop')
        self.TeleTopic = rospy.get_param('~TeleTopic')

        if not rospy.has_param('~accsp'):
            rospy.set_param('~accsp', 0.005)
        self.accsp = rospy.get_param('~accsp')

        self.cur_cmd = None
        self.safty_sp = Twist()

    def SaftyCB(self, message):
        if message.left == 1 or message.center == 2 or message.right == 4:
            if self.cur_cmd != None:
                if round(self.cur_cmd.linear.x, 2) != 0 or round(self.cur_cmd.angular.z, 2) != 0:
                    self.safty_sp.linear.x += CMDLib._sign(-self.cur_cmd.linear.x)*self.accsp
                    self.safty_sp.angular.z += CMDLib._sign(-self.cur_cmd.linear.x)*self.accsp
                    if self.safty_sp.linear.x <= 0 :
                        self.safty_sp.linear.x = 0
                    if self.safty_sp.angular.z <= 0:
                        self.safty_sp.angular.z = 0
                    self.Pub(self.safty_sp)
                    self.safty_sp = Twist()
                else:
                    rospy.loginfo('zero cmd input')
            else:
                rospy.loginfo('none cmd input')

    def Pub(self, data):
        pub = rospy.Publisher(self.TeleTopic, Twist, queue_size=1)
        pub.publish(data)

    def NaviCB(self, message):
        self.cur_cmd = message

if __name__ == '__main__':
    rospy.init_node('Safty_Control')
    try:
        rospy.loginfo("initialization system")
        Safty_Control()
        ClearParams()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")