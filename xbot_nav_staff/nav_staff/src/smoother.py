#!/usr/bin/env python
# coding=utf-8

"""
cmd vector smoother

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import Twist

class smoother():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.MotionTopice, Twist, self.CMDCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~MotionTopice'):
            rospy.set_param('~MotionTopice', 'cmd_vel_mux/input/smoother')
        self.MotionTopice = rospy.get_param('~MotionTopice')
        if not rospy.has_param('~CmdTopice'):
            rospy.set_param('~CmdTopice', 'cmd_vel_mux/input/navi')
        self.MotionTopice = rospy.get_param('~MotionTopice')
        self.CmdTopic = rospy.get_param('~CmdTopice')
        self.pre_cmd = None

    def CMDCB(self, cmd):
        # print 'cmd: ',cmd.linear.x
        if self.pre_cmd == None:
            self.pre_cmd = cmd
        if abs(round(self.pre_cmd.linear.x - cmd.linear.x, 2)) >= 0.01:
            self.pre_cmd.linear.x += self._sign(self.pre_cmd.linear.x - cmd.linear.x)* 0.01
            # print 'pre_cmd: ',self.pre_cmd.linear.x, '\n'
        else:
            self.pre_cmd = cmd
        self.PUB(self.pre_cmd)

    def _sign(self, data):
        if data > 0:
            return -1
        elif data < 0:
            return 1
        else:
            return 0

    def PUB(self, data):
        publish = rospy.Publisher(self.CmdTopic, Twist, queue_size=1)
        publish.publish(data)
        # print data,'\n'

if __name__=='__main__':
     rospy.init_node('Cmd_Smoother')
     try:
         rospy.loginfo( "initialization system")
         smoother()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")
