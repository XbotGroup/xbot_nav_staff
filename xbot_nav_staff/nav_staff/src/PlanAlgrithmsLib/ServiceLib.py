#!/usr/bin/env python
# coding=utf-8
"""
modified rospy core service API for amcl

Copyright (c) 2016 Xu Zhihao (Howe).  All rights reserved.

This program is free software; you can redistribute it and/or modify

This programm is tested on kuboki base turtlebot.

"""
import rospy
import socket
import rosgraph
import time
import rosgraph.names
import rosgraph.network
from rospy.exceptions import ROSInterruptException


def wait_for_service_D(service, timeout = None):
    master = rosgraph.Master(rospy.names.get_caller_id())
    if timeout == 0:
        rospy.loginfo('timeout must be non-zero value, setting timeout to default value')
        return False
    else:
        resolved_name = rospy.names.resolve_name(service)
        rospy.loginfo ('resolved_name: ' + resolved_name)
        if timeout:
            timeout_time = time.time() + timeout
            while not rospy.core.is_shutdown() and time.time() < timeout_time:
                try:
                    if contact_service(resolved_name, master, timeout_time - time.time()):
                        return True
                    time.sleep(0.3)
                except KeyboardInterrupt:
                    rospy.loginfo('wait_for_service: received keyboard interrupt')
                    raise
                except:
                    rospy.loginfo("wait_for_service(%s): service not actually up failed to contact [%s], will keep trying" % (resolved_name, uri))
                    return False
            if rospy.core.is_shutdown():
                raise ROSInterruptException("rospy shutdown")
            else:
                # raise ROSException("timeout exceeded while waiting for service %s" % resolved_name)\
                rospy.loginfo("timeout exceeded while waiting for service %s" % resolved_name)
                return False

        else:  # timeout is None
            while not rospy.core.is_shutdown():
                try:
                    if contact_service(resolved_name, master):
                        return True
                    time.sleep(0.3)
                except KeyboardInterrupt:
                    rospy.loginfo('wait_for_service: received keyboard interrupt')
                    raise
                except:
                    rospy.loginfo("wait_for_service(%s): will keep trying" %resolved_name)
                    return False
            if rospy.core.is_shutdown():
                raise ROSInterruptException("rospy shutdown")

def contact_service(resolved_name, master, timeout = 10):
    try:
        uri = master.lookupService(resolved_name)
        #print 'uri: ',uri
    except rosgraph.MasterException:
        #print "MasterException"
        return False

    addr = rospy.core.parse_rosrpc_uri(uri)
    if rosgraph.network.use_ipv6():
        s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        # we always want to timeout just in case we're connecting
        # to a down service.
        s.settimeout(timeout)
        rospy.logdebug('connecting to ' + str(addr))
        s.connect(addr)
        h = {'probe': '1', 'md5sum': '*',
             'callerid': rospy.core.get_caller_id(),
             'service': resolved_name}
        rosgraph.network.write_ros_handshake_header(s, h)
        return True
    finally:
        if s is not None:
            s.close()
