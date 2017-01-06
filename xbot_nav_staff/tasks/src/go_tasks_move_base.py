#!/usr/bin/env python
#coding=utf-8
"""
task manager

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""

import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from threading import Lock
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
import move_reference
import actionlib
import collections
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray


CurrentGoal = MoveBaseGoal()

class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_odom_topic')
        rospy.delete_param('~use_marker_topic')
        rospy.delete_param('~use_status_topic')
        rospy.delete_param('~task_number')


class go_task():
    def __init__(self):
        self.define()
        rospy.Subscriber(self.OdomTopic, PoseStamped, self.OdomCB)
        rospy.Subscriber(self.MarkerTopic, Marker, self.GoalCB)
        rospy.Subscriber(self.StatusTopic, GoalStatusArray, self.StatusCB)
        rospy.spin()

    def define(self):
        if not rospy.has_param('~use_odom_topic'):
            rospy.set_param('~use_odom_topic', '/robot_position_in_map')
        self.OdomTopic = rospy.get_param('~use_odom_topic')

        if not rospy.has_param('~use_marker_topic'):
            rospy.set_param('~use_marker_topic', '/ui_marker')
        self.MarkerTopic = rospy.get_param('~use_marker_topic')

        if not rospy.has_param('~use_status_topic'):
            rospy.set_param('~use_status_topic', '/move_base/status')
        self.StatusTopic = rospy.get_param('~use_status_topic')

        if not rospy.has_param('~task_number'):
            rospy.set_param('~task_number', 1)
        self.task_number = rospy.get_param('~task_number')

        self.locker = Lock()

        self.tasks = collections.deque(maxlen = self.task_number)

        self.active_state = {
            GoalStatus.ACTIVE: 'ACTIVE: TASK goal is currently being processed by the server, PLEASE WAIT for SUCCEEDED',
            GoalStatus.SUCCEEDED: 'SUCCEEDED: TASK goal was achieved successfully by the server, PLEASE SEND A NEW GOAL'
        }

        self.error_state = {
            #GoalStatus.PENDING: 'PENDING: TASK goal has yet to be processed by the server, PLEASE WAIT',
            #GoalStatus.PREEMPTED: 'PREEMPTED: TASK goal received a cancel request after it started executing and has since completed its execution, PLEASE SEND A NEW GOAL',
            GoalStatus.ABORTED: 'ABORTED: TASK goal was aborted during execution by the server due to some failure, PLEASE SEND A NEW GOAL',
            GoalStatus.REJECTED: 'REJECTED: TASK goal was rejected by the server without being processed, because the goal was unattainable or invalid, PLEASE SEND A NEW GOAL',
            #GoalStatus.PREEMPTING: 'PREEMPTING: TASK goal received a cancel request after it started executing and has not yet completed execution, PLEASE SEND A NEW GOAL',
            #GoalStatus.RECALLING: 'RECALLING: TASK goal received a cancel request before it started executing, but the server has not yet confirmed that the goal is canceled, PLEASE SEND A NEW GOAL',
            GoalStatus.RECALLED: 'RECALLED: TASK goal received a cancel request before it started executing and was successfully cancelled, PLEASE SEND A NEW GOAL',
            GoalStatus.LOST: 'LOST: TASK is determineed that a goal is LOST. This should not be sent over the wire by an server, PLEASE SEND A NEW GOAL'
        }

        self.ns = 'move_base'

        self.move_base = actionlib.SimpleActionClient(self.ns, MoveBaseAction)

        self.move_base.wait_for_server()


    def StatusCB(self, staus):
        state_list = staus.status_list
        if state_list != []:
            for i in state_list:
                if i.status in self.error_state.keys():
                    #pub_cancel = rospy.Publisher(rospy.remap_name(self.ns) + '/cancel', GoalID, queue_size=1)
                    #pub_cancel.publish(i.goal_id)
                    self.move_base.cancel_all_goals()
                if i.status in self.active_state.keys():
                    rospy.loginfo('task in progress plz wait')
                    pass


    def OdomCB(self, data):
        with self.locker:
            pose = data.pose
            rospy.loginfo(str(len(self.tasks)))
            if len(self.tasks):
                marker = self.tasks.pop()
                global CurrentGoal
                CurrentGoal = marker.points[0]
                task_goal = self.find_goal(pose.position, CurrentGoal)
                task_goal.header.seq = marker.header.seq
                task_goal.header.stamp = rospy.Time.now()
                # pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
                # pub_goal.publish(task_goal)
                goal = MoveBaseGoal()
                goal.target_pose = task_goal
                self.move_base.send_goal(goal)
                self.move_base.wait_for_result(rospy.Duration(10))


    def GoalCB(self, data):
        with self.locker:
            #self.move_base.cancel_all_goals()
            marker_point = Marker()
            marker_point = data
            self.tasks.append(marker_point)

    def find_goal(self, initposition, taskposition):
        task_goal = PoseStamped()
        angle = move_reference.angle_generater(taskposition, initposition)
        (gqx, gqy, gqz, gqw) = move_reference.angle_to_quat(angle)
        try:
            task_goal.header.frame_id = self.marker_point.header.frame_id
        except:
            task_goal.header.frame_id = 'map'
        task_goal.pose.position = taskposition
        task_goal.pose.orientation.x = gqx
        task_goal.pose.orientation.y = gqy
        task_goal.pose.orientation.z = gqz
        task_goal.pose.orientation.w = gqw
        return task_goal


if __name__ == '__main__':
     rospy.init_node("go_task", anonymous=True)
     try:
      rospy.loginfo ("initialization system")
      go_task()
      ClearParams()
      rospy.loginfo ("process done and quit")
     except rospy.ROSInterruptException:
      rospy.loginfo("node terminated.")