#!/usr/bin/env python
# coding:utf-8

# Software License Agreement (BSD License)
#
#  Copyright (c) 2019, Kawasaki Heavy Industries, LTD.
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Willow Garage nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.

PKG = 'khi_robot_test'
import roslib; roslib.load_manifest(PKG)

import copy
import math
import sys
import unittest
import rospy
import geometry_msgs.msg
from khi_robot_msgs.srv import *

if rospy.has_param('/test_group_name'):
    gn = '/' + rospy.get_param('/test_group_name')
else:
    gn = ''
service = gn + '/khi_robot_command_service'

def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service,KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException, e:
        rospy.loginfo('Service call failed: %s', e)

def get_driver_state():
    ret = cmdhandler_client('driver' , 'get_status')
    return ret

class TestKhiRobotControl(unittest.TestCase):
    def test_program(self):
        # ROS -> HOLDED
        cmdhandler_client('driver', 'hold')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('HOLDED', ret.cmd_ret)

        # ROS -> INACTIVE
        cmdhandler_client('as', 'hold 1:')
        rospy.sleep(3)
        ret = cmdhandler_client('as', 'type switch(cs)')
        self.assertEqual(0, int(ret.cmd_ret))
        ret = get_driver_state()
        self.assertEqual('INACTIVE', ret.cmd_ret)

        # execute robot program by AS
        cmdhandler_client('as', 'zpow on')
        cmdhandler_client('as', 'execute 1: rb_rtc1,-1')
        ret = cmdhandler_client('as', 'type switch(cs)')
        self.assertEqual(-1, int(ret.cmd_ret))
        ret = get_driver_state()
        self.assertEqual('INACTIVE', ret.cmd_ret)

        # ROS -> ACTIVE
        cmdhandler_client('driver', 'restart')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

    def test_signal(self):
        size  = 512
        offset = [0, 2000, 2000]

        cmdhandler_client('as', 'ZINSIG ON')
        cmdhandler_client('as', 'RESET')

        for i in range(size):
            for j in range(3):
                rospy.loginfo('SIGNAL %d', i+1+offset[j])
                # positive
                set_cmd = 'set_signal ' + str(i+1+offset[j])
                cmdhandler_client('driver', set_cmd)
                get_cmd = 'get_signal ' + str(i+1+offset[j])
                ret = cmdhandler_client('driver', get_cmd)
                self.assertEqual('-1', ret.cmd_ret)

                # negative
                set_cmd = 'set_signal -' + str(i+1+offset[j])
                cmdhandler_client('driver', set_cmd)
                get_cmd = 'get_signal ' + str(i+1+offset[j])
                ret = cmdhandler_client('driver', get_cmd)
                self.assertEqual('0', ret.cmd_ret)

    def test_variable(self):
        cmdhandler_client('as', 'test_val = 1')
        ret = cmdhandler_client('as', 'type test_val')
        self.assertEqual(1, int(ret.cmd_ret))

if __name__ == '__main__':
    import rostest
    rospy.init_node("test_khi_robot_control_node")
    cmdhandler_client('driver', 'restart')
    rostest.rosrun(PKG, 'test_khi_robot_control', TestKhiRobotControl)