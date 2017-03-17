#!/usr/bin/env python
# coding=utf-8
"""
AMCL TF 转换

map -- odom -- base

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from PlanAlgrithmsLib import ServiceLib
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf
from geometry_msgs.msg import PoseStamped
from threading import Lock
import copy
from nav_msgs.msg import Odometry


ServerCheck = False
First = True
Locker = Lock()
position = tuple()
orientation = tuple()


class ClearParams:
    def __init__(self):
        rospy.delete_param('~use_map_topic')
        rospy.delete_param('~use_scan_topic')
        rospy.delete_param('~use_initialpose_topic')
        rospy.delete_param('~AmclMapTopic')
        rospy.delete_param('~target_frame')
        rospy.delete_param('~source_frame')

        rospy.delete_param('~initial_position_x')
        rospy.delete_param('~initial_position_y')
        rospy.delete_param('~initial_position_z')

        rospy.delete_param('~initial_orientation_x')
        rospy.delete_param('~initial_orientation_y')
        rospy.delete_param('~initial_orientation_z')
        rospy.delete_param('~initial_orientation_w')

class AMCL():
    def __init__(self):
        self.define()
        self.RequestMap()
        rospy.Subscriber(self.InitialposeTopic, PoseWithCovarianceStamped, self.HandleInitialPoseMessage)
        rospy.Subscriber(self.MapTopic, OccupancyGrid, self.HandleMapMessage)
        rospy.Subscriber(self.ScanTopic, LaserScan, self.HandleLaserMessage)
        rospy.Subscriber(self.OdomTopic, Odometry, self.HandleOdomMessage)
        rospy.Timer(self.amcl_frequency, self.PubamclCB)
        rospy.spin()

    def PubamclCB(self, event):
        br = tf.TransformBroadcaster()
        global position
        global orientation
        position = (0,0,0)
        orientation = (0,0,0,1)
        # br.sendTransform((position, orientation, rospy.Time.now(), self.source_frame, self.global_frame)

    def RequestMap(self):
        global ServerCheck
        global First
        while not ServerCheck:
            ReadMapService = rospy.ServiceProxy('/static_map', GetMap)
            ServerCheck = ServiceLib.wait_for_service_D('/static_map',1)
            rospy.loginfo('Request for map failed; trying again')
            rospy.sleep(0.3)
        response = ReadMapService()
        rospy.loginfo('Map recieved')
        if First:
         self.HandleMapMessage(response.map)
         rospy.loginfo('map info: ' + str(response.map.info.width) + ' X ' + str(response.map.info.height) + ', pix: ' + str(round(response.map.info.resolution,3)))
         First = False

    def HandleMapMessage(self, map_msg):
        with Locker:
            self.internal_map = self.ConvertMap(map_msg)
            self.Pubmap(map_msg)
            self.apply_pose()


    def Pubmap(self, map_msg):
        pub = rospy.Publisher(self.AmclMapTopic, OccupancyGrid, queue_size=1)
        map = OccupancyGrid()
        map = map_msg
        pub.publish(map)
        #print len(self.map_.data), '\nmap_.info:\n', self.map_.info, '\nmap_.header\n',self.map_.header

    def ConvertMap(self, map_msg):
        #Convert an OccupancyGrid map message into the internal representation.  This allocates a map_t and returns it.
        map = OccupancyGrid()
        map.data = list(copy.deepcopy(map_msg.data))
        for i in range(map_msg.info.width * map_msg.info.height):
            if map_msg.data[i] == 0:
                map.data[i] = -1
            elif map_msg.data[i] == 100:
                map.data[i] = 1
            else:
                map.data[i] = 0
        return map

    def HandleInitialPoseMessage(self, init_msg):
        with Locker:
            if init_msg.header.frame_id == '':
                rospy.logwarn('Received initial pose with empty frame_id.  You should always supply a frame_id.')
            elif init_msg.header.frame_id != self.map_.header.frame_id:
                rospy.logwarn('Ignoring initial pose in frame' + str(init_msg.header.frame_id) +';' + 'initial poses must be in the global frame' + str(self.map_.header.frame_id))
            else:
                print 'get initial pose'
                init_pose = PoseStamped()
                init_pose.header = init_msg.header
                init_pose.pose = init_msg.pose.pose
                rospy.loginfo('updating robot initial pose')
                self.PubInitPose(init_pose)
                self.init_pose = copy.deepcopy(init_pose)

    def apply_pose(self):
        #用来update robot position in map
        self.listener.waitForTransform(self.target_frame, self.source_frame, rospy.Time.now(), rospy.Duration(0.1))
        (trans, rot) = self.listener.lookupTransform(self.target_frame, self.source_frame, rospy.Time.now())

        self.init_pose.pose.position.x = trans[0]
        self.init_pose.pose.position.y = trans[1]
        self.init_pose.pose.position.z = trans[2]

        self.init_pose.pose.orientation.x = rot[0]
        self.init_pose.pose.orientation.y = rot[1]
        self.init_pose.pose.orientation.z = rot[2]
        self.init_pose.pose.orientation.w = rot[3]
        return

    def PubInitPose(self, pose_msg):
        pose_msg.header.stamp = rospy.Time.now()
        pub = rospy.Publisher('/set_pose', PoseStamped, queue_size=1)
        pub.publish(pose_msg)
        self.current_pose = PoseStamped()
        self.current_pose = copy.deepcopy(pose_msg)

    def HandleLaserMessage(self, data):
        pass

    def HandleOdomMessage(self, data):

        self.current_pose.pose.position.x += data.pose.pose.position.x
        self.current_pose.pose.position.y += data.pose.pose.position.y
        self.current_pose.pose.position.z += data.pose.pose.position.z

        self.current_pose.pose.orientation.x += data.pose.pose.orientation.x
        self.current_pose.pose.orientation.y += data.pose.pose.orientation.y
        self.current_pose.pose.orientation.z += data.pose.pose.orientation.z
        self.current_pose.pose.orientation.w += data.pose.pose.orientation.w

    def define(self):
        if not rospy.has_param('~use_map_topic'):
            rospy.set_param('~use_map_topic', '/map_raw')
        self.MapTopic = rospy.get_param('~use_map_topic')

        if not rospy.has_param('~use_scan_topic'):
            rospy.set_param('~use_scan_topic', '/scan')
        self.ScanTopic = rospy.get_param('~use_scan_topic')

        if not rospy.has_param('~use_initialpose_topic'):
            rospy.set_param('~use_initialpose_topic', '/initialpose')
        self.InitialposeTopic = rospy.get_param('~use_initialpose_topic')

        # if not rospy.has_param('~AmclMapTopic'):
        #     rospy.set_param('~AmclMapTopic', '/amcl_map')
        # self.AmclMapTopic = rospy.get_param('~AmclMapTopic')

        if not rospy.has_param("~target_frame"):
            rospy.set_param("~target_frame", "/odom")
        self.target_frame = rospy.get_param("~target_frame")

        if not rospy.has_param("~source_frame"):
            rospy.set_param("~source_frame", "/base_link")
        self.source_frame = rospy.get_param("~source_frame")

        if not rospy.has_param("~global_frame"):
            rospy.set_param("~global_frame", "/map")
        self.global_frame = rospy.get_param("~global_frame")

        if not rospy.has_param("~initial_position_x"):
            rospy.set_param("~initial_position_x", 0)
        init_position_x = rospy.get_param("~initial_position_x")

        if not rospy.has_param("~initial_position_y"):
            rospy.set_param("~initial_position_y", 0)
        init_position_y = rospy.get_param("~initial_position_y")

        if not rospy.has_param("~initial_position_z"):
            rospy.set_param("~initial_position_z", 0)
        init_position_z = rospy.get_param("~initial_position_z")

        if not rospy.has_param("~initial_orientation_x"):
            rospy.set_param("~initial_orientation_x", 0)
        init_orientation_x = rospy.get_param("~initial_orientation_x")

        if not rospy.has_param("~initial_orientation_y"):
            rospy.set_param("~initial_orientation_y", 0)
        init_orientation_y = rospy.get_param("~initial_orientation_y")

        if not rospy.has_param("~initial_orientation_z"):
            rospy.set_param("~initial_orientation_z", 0)
        init_orientation_z = rospy.get_param("~initial_orientation_z")

        if not rospy.has_param("~initial_orientation_w"):
            rospy.set_param("~initial_orientation_w", 1)
        init_orientation_w = rospy.get_param("~initial_orientation_w")

        if not rospy.has_param("~AMCL_frequency"):
            rospy.set_param("~AMCL_frequency", 10)
        AMCL_frequency = rospy.get_param("~AMCL_frequency")

        self.amcl_frequency = 1.0/AMCL_frequency

        self.listener=tf.TransformListener()

        self.init_pose = PoseStamped()

        self.init_pose.pose.position.x = init_position_x
        self.init_pose.pose.position.y = init_position_y
        self.init_pose.pose.position.z = init_position_z

        self.init_pose.pose.orientation.x = init_orientation_x
        self.init_pose.pose.orientation.y = init_orientation_y
        self.init_pose.pose.orientation.z = init_orientation_z
        self.init_pose.pose.orientation.w = init_orientation_w

        self.PubInitPose(self.init_pose)

if __name__=='__main__':
    rospy.init_node('amcl_adapted')
    try:
        rospy.loginfo( "initialization system")
        AMCL()
        ClearParams()
        rospy.loginfo("process done and quit" )
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
