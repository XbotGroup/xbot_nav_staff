#!/usr/bin/env python
# coding=utf-8
"""
test plan 算法库的测试程序

Copyright (c) 2017 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from std_msgs.msg import Bool

# speaker simulation
class tester1():
    def __init__(self):
        self.Connected = False
        self.StopRun = False
        self.pub = rospy.Publisher('/speak_done', Bool, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.Connect)
        rospy.Timer(rospy.Duration(0.1), self.pubCB)
        rospy.spin()

    def Connect(self, event):
        if not self.Connected:
            print 'try connect'
            self.pub.publish(False)
            self.Connected = rospy.wait_for_message('/StopRun_run', Bool)

    def pubCB(self, event):
        if self.Connected:
            print 'speaker sim'
            rospy.sleep(2.0)
            self.pub.publish(True)



if __name__=='__main__':
     rospy.init_node('Plan_tester_pub2')
     try:
         rospy.loginfo( "initialization system")
         tester1()
         rospy.loginfo("process done and quit" )
     except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")

