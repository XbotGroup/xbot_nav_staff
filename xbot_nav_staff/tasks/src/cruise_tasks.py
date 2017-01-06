#!/usr/bin/env python
#coding=utf-8
"""

Copyright (c) 2015 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot. 

"""
import rospy
import getpass
import actionlib
import actions_reference
import move_reference
import numpy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry

"""class cruise_modle():
 def __init__(self):
  Plist=[]
  Current_position = rospy.wait_for_message("turtlebot_position_in_map", Pose)
  Plist.append(Current_position.position)
  self.loading(Plist)

 def loading(self, Plist):
  data=raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position =  rospy.wait_for_message("clicked_point",PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    Plist.append(position.point)
   actions_reference.cruise(Plist)
  except:
   self.loading(Plist)"""
   
   
class cruise_modle():
 def __init__(self):
  self.define()
  rospy.Subscriber("turtlebot_position_in_map", Pose, self.OdomCB)
  #rospy.Subscriber('/move_base/DWAPlannerROS/global_plan', Path, self.MultiTasksPlanCB)
  rospy.Subscriber('/move_base/TrajectoryPlannerROS/global_plan', Path, self.MultiTasksPlanCB)# 仿真
 #rospy.Timer(rospy.Duration(0.5), self.GoalUpdate)
  if not self.loaded:
   self.Plist = self.loading()
   self.SendGoal()
  rospy.spin()
  
 def define(self):
  self.Plist = []
  self.loaded = False
  self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
  self.move_base.wait_for_server()
  self.count = 0
  self.goal = MoveBaseGoal()
  self.goal.target_pose.header.frame_id = 'map'    
  self.seq_num = 0
  self.PreOdom = Pose()
  
  
 def Order_Switcher(self):
  #print 'self.count',self.count
  if self.count == len(self.Plist):
   self.Plist.reverse()
   self.count = 0

 def Goal_Switcher(self): 
  if len(self.Plist) > 0: 
   if self.Achieve(self.Plist[self.count]):
    self.move_base.cancel_goal()
    self.SendGoal()
    self.count += 1
    self.Order_Switcher()
    
 def SendGoal(self):
  self.goal.target_pose.pose.position = self.Plist[self.count]
  self.goal.target_pose.header.seq = self.seq_num
  self.goal.target_pose.header.stamp = rospy.Time.now()
  angle = move_reference.angle_generater(self.Plist[self.count], self.Current_position.position)
  (self.goal.target_pose.pose.orientation.x, self.goal.target_pose.pose.orientation.y, self.goal.target_pose.pose.orientation.z, self.goal.target_pose.pose.orientation.w) = move_reference.angle_to_quat(angle)
  #print 'send current goal: ', self.count , '\n', self.goal
  self.move_base.send_goal(self.goal)
  self.seq_num += 1
  
 def GoalUpdate(self, event):
  if len(self.Plist) > 0: 
   Ex = round(self.PreOdom.position.x - self.Current_position.position.x, 2) == 0.0
   Ey = round(self.PreOdom.position.y - self.Current_position.position.y, 2) == 0.0
   if Ex and Ey:
    #self.move_base.cancel_goal()
    self.SendGoal()
  
 def Achieve(self, goal):
  X_Error = round(round(goal.x, 2) - round(self.Current_position.position.x, 2), 2) <= 0.1
  Y_Error = round(round(goal.y, 2) - round(self.Current_position.position.y, 2), 2) <= 0.1
  print round(round(goal.x, 2) - round(self.Current_position.position.x, 2), 2), round(round(goal.y, 2) - round(self.Current_position.position.y, 2), 2), self.count
  if X_Error and Y_Error:
   return True
  else:
   return False
  
  
 def MultiTasksPlanCB(self, path):  
  path_poses = path.poses
  path_num = len(path_poses)
  Sessor = 0.5
  if path_num > 100:
   rospy.loginfo('long path model' + '%s'%len(path_poses))
   self.move_base.cancel_goal()
   rospy.loginfo('cancel old goal and create a new goal')
   new_goal = MoveBaseGoal()
   new_point = path_poses[int(path_num * Sessor)].pose.position
   new_angle = move_reference.angle_generater(new_point, path_poses[int(path_num * Sessor) + 1].pose.position)
   new_goal.target_pose.header.frame_id = 'map'    
   new_goal.target_pose.pose.position = new_point
   (new_goal.target_pose.pose.orientation.x, new_goal.target_pose.pose.orientation.y, new_goal.target_pose.pose.orientation.z, new_goal.target_pose.pose.orientation.w) = move_reference.angle_to_quat(new_angle)
   new_goal.target_pose.header.seq = self.seq_num
   new_goal.target_pose.header.stamp = rospy.Time.now()
   self.move_base.send_goal(new_goal)
   self.seq_num += 1
  """else:
   self.goal.target_pose.header.seq = self.seq_num
   self.goal.target_pose.header.stamp = rospy.Time.now()
   self.move_base.send_goal(self.goal)
   self.seq_num += 1"""

 def OdomCB(self, data):
  self.Current_position = data
  self.Goal_Switcher()
  
 def loading(self):
  Plist = []
  data = raw_input('请输入巡航的目标点数量： ')
  try:
   num = int(data)
   for i in range(num):
    rospy.loginfo ('预备录入第%s个目标点'%(i+1))
    position =  rospy.wait_for_message("clicked_point",PointStamped)
    rospy.loginfo ('已录入第%s个目标点'%(i+1))
    Plist.append(position.point)
   self.loaded = True
   return Plist
  except:
   self.loading()

     
if __name__ == '__main__':
 rospy.init_node('cruise_tasks')
 try:
  rospy.loginfo( "initialization system")
  cruise_modle()
  rospy.loginfo( "process done and quit")
 except rospy.ROSInterruptException:
  rospy.loginfo("follower node terminated.")

