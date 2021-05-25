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
import moveit_commander
import geometry_msgs.msg
from khi_robot_msgs.srv import *
from rospkg import RosPack, ResourceNotFound

if rospy.has_param('/test_group_name'):
    gn = '/' + rospy.get_param('/test_group_name')
else:
    gn = ''
service = gn+'/khi_robot_command_service'
planner = 'RRTConnectkConfigDefault'

class KhiRobot:
    arm_name = ''
    arm_num = 1
    max_jt = 6
    group = gn+'/manipulator'
    min_pos_list = []
    max_pos_list = []
    max_vel_list = []
    max_acc_list = []
    base_pos_list = []

    def __init__(self):
        self.min_pos_list = []
        self.max_pos_list = []
        self.max_vel_list = []
        self.max_acc_list = []
        self.arm_name = rospy.get_param(gn+'/khi_robot_param/robot')
        limits = rospy.get_param(gn+'/'+self.arm_name+'/joint_limits')
        if self.arm_name == 'WD002N':
            self.arm_num = 2
            self.max_jt = 8
            self.group = 'botharms'
            self.base_pos_list = [ -90*math.pi/180, 0, 0, 0, 90*math.pi/180, 0, 0, 0 ]
            for jt in range(self.max_jt):
                if jt < 4:
                    self.min_pos_list.append(limits['lower_joint'+str(jt+1)]['min_position'])
                    self.max_pos_list.append(limits['lower_joint'+str(jt+1)]['max_position'])
                    self.max_acc_list.append(limits['lower_joint'+str(jt+1)]['max_acceleration'])
                    self.max_vel_list.append(limits['lower_joint'+str(jt+1)]['max_velocity'])
                else:
                    self.min_pos_list.append(limits['upper_joint'+str(jt-3)]['min_position'])
                    self.max_pos_list.append(limits['upper_joint'+str(jt-3)]['max_position'])
                    self.max_acc_list.append(limits['upper_joint'+str(jt-3)]['max_acceleration'])
                    self.max_vel_list.append(limits['upper_joint'+str(jt-3)]['max_velocity'])
        else:
            self.arm_num = 1
            self.max_jt = 6
            self.group = 'manipulator'
            self.base_pos_list = [ 90*math.pi/180, 0, 90*math.pi/180, 0, 90*math.pi/180, 0 ]
            for jt in range(self.max_jt):
                self.min_pos_list.append(limits['joint'+str(jt+1)]['min_position'])
                self.max_pos_list.append(limits['joint'+str(jt+1)]['max_position'])
                self.max_vel_list.append(limits['joint'+str(jt+1)]['max_velocity'])
                self.max_acc_list.append(limits['joint'+str(jt+1)]['max_acceleration'])

    def get_pos_list(self, ano, jt, type):
        jt_list = copy.deepcopy(self.base_pos_list)
        if self.arm_name.startswith('RS'):
            if type == 'min':
                if jt+1 == 2:
                    jt_list[2] = -130*math.pi/180
                    jt_list[0] = -135*math.pi/180
                jt_list[jt] = self.min_pos_list[jt]
            else:
                if jt+1 == 2:
                    jt_list[2] = 130*math.pi/180
                    jt_list[0] = -135*math.pi/180
                jt_list[jt] = self.max_pos_list[jt]
        elif self.arm_name.startswith('WD'):
            if type == 'min':
                if jt+1 == 5:
                    jt_list[0] = -170*math.pi/180
                jt_list[jt] = self.min_pos_list[jt]
            else:
                if jt+1 == 1:
                    jt_list[4] = 220*math.pi/180
                elif jt+1 == 5:
                    jt_list[0] = 170*math.pi/180
                jt_list[jt] = self.max_pos_list[jt]

        return jt_list

def cmdhandler_client(type_arg , cmd_arg):
    rospy.wait_for_service(service)
    try:
        khi_robot_command_service = rospy.ServiceProxy(service,KhiRobotCmd)
        resp1 = khi_robot_command_service(type_arg, cmd_arg)
        return resp1
    except rospy.ServiceException as e:
        rospy.loginfo('Service call failed: %s', e)

def get_driver_state():
    ret = cmdhandler_client('driver' , 'get_status')
    return ret

def plan_and_execute(mgc,jt,num,timeout):
    for cnt in range(timeout):
        # checking the version of the 'moveit_core' package
        (major,minor,bugfix) = get_package_version('moveit_commander')
        if major > 1 or ( major == 1 and minor >= 1 ):
            (flag, trajectory, plannint_time, error_code) = mgc.plan()
            if flag is False:
                rospy.loginfo('JT%d-%d: cannot be planned %d times', jt+1, num+1, cnt+1)
            else:
                ret = mgc.execute(trajectory)
                if ret is False:
                    rospy.loginfo('JT%d-%d: cannot be executed %d times', jt+1, num+1, cnt+1)
                else:
                    return True
        else:
            plan = mgc.plan()
            if plan is False:
                rospy.loginfo('JT%d-%d: cannot be planned %d times', jt+1, num+1, cnt+1)
            else:
                ret = mgc.execute(plan)
                if ret is False:
                    rospy.loginfo('JT%d-%d: cannot be executed %d times', jt+1, num+1, cnt+1)
                else:
                    return True

        if cnt == timeout-1:
            rospy.loginfo('timeout')
            return False
    return False

def get_package_version(packagename):
    rp = RosPack()
    try:
        manifest = rp.get_manifest(packagename)
        return tuple(map(int,manifest.version.split('.')))
    except ResourceNotFound:
        return tuple()

class TestKhiRobotControl(unittest.TestCase):
    def test_position_velocity(self):
        ##############################set parameter############################
        accuracy_jt  = 0.01    # joint accuracy
        #accuracy_pos = 0.01    # position accuracy
        #accuracy_ori = 0.01    # orientation accuracy

        max_vel = 1.0           # max velocity scale
        max_acc = 1.0           # max acceleration scale
        min_vel = 0.5           # min velocity scale
        min_acc = 0.5           # min acceleration scale

        cyc_num = 3             # reperat num
        timeout = 10            # timeout num
        retcode = 0
        ########################################################################

        ######################################### set range of move #####################################################
        khi_robot = KhiRobot()

        # RobotCommander
        rc = moveit_commander.RobotCommander()

        # MoveGroupCommander
        mgc = moveit_commander.MoveGroupCommander(khi_robot.group)

        # mgc setting
        mgc.set_planner_id(planner)
        mgc.set_goal_joint_tolerance(accuracy_jt)
        #mgc.set_goal_position_tolerance(accuracy_pos)
        #mgc.set_goal_orientation_tolerance(accuracy_ori)

        ret = get_driver_state()
        if ret.cmd_ret == 'ERROR':
            cmdhandler_client('driver', 'restart')
            rospy.sleep(3)

        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

        # move each joint
        for jt in range(khi_robot.max_jt):
            if khi_robot.arm_name == 'WD002N':
                if jt < 4:
                    ano = 0
                else:
                    ano = 1
            else:
                ano = 0

            # min position/velocity/acceleration
            mgc.set_max_velocity_scaling_factor(min_vel)
            mgc.set_max_acceleration_scaling_factor(min_acc)
            jt_list = khi_robot.get_pos_list(ano, jt, 'min')
            rospy.loginfo('JT%d  : base', jt+1)
            mgc.set_joint_value_target(jt_list)
            self.assertTrue(plan_and_execute(mgc,jt,-1,timeout))

            # max velocity/acceleration
            mgc.set_max_velocity_scaling_factor(max_vel)
            mgc.set_max_acceleration_scaling_factor(max_acc)

            # do repeat moving
            for num in range(cyc_num):
                # max position
                jt_list = khi_robot.get_pos_list(ano, jt, 'max')
                rospy.loginfo('JT%d-%d:  max ', jt+1, num+1)
                mgc.set_joint_value_target(jt_list)
                retcode = plan_and_execute(mgc,jt,num,timeout)
                self.assertTrue(retcode)
                if retcode == False:
                    rospy.loginfo('JT%d-%d:  max faild.', jt+1, num+1)
                    break
                now_jt_list = mgc.get_current_joint_values()
                self.assertAlmostEqual(jt_list[jt], now_jt_list[jt], delta = accuracy_jt*2)

                # min position
                jt_list = khi_robot.get_pos_list(ano, jt, 'min')
                rospy.loginfo('JT%d-%d:  min', jt+1, num+1)
                mgc.set_joint_value_target(jt_list)
                retcode = plan_and_execute(mgc,jt,1,timeout)
                self.assertTrue(retcode)
                if retcode == False:
                    rospy.loginfo('JT%d-%d:  min faild.', jt+1, num+1)
                    break
                now_jt_list = mgc.get_current_joint_values()
                self.assertAlmostEqual(jt_list[jt], now_jt_list[jt], delta = accuracy_jt*2)

                # state check
                ret = get_driver_state()
                self.assertEqual('ACTIVE', ret.cmd_ret)
                if ret.cmd_ret != 'ACTIVE':
                    rospy.loginfo('JT%d-%d:  not active', jt+1, num+1)
                    break

            # finish repeat moveing
            ret = get_driver_state()
            self.assertEqual('ACTIVE', ret.cmd_ret)
            if ret.cmd_ret != 'ACTIVE':
                break

    def test_state(self):
        cmdhandler_client('driver', 'restart')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

        cmdhandler_client('driver', 'hold')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('HOLDED', ret.cmd_ret)

        cmdhandler_client('driver', 'restart')
        rospy.sleep(3)
        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

if __name__ == '__main__':
    import rostest
    cmdhandler_client('driver', 'restart')
    rospy.init_node("test_khi_robot_control_node")
    rostest.rosrun(PKG, 'test_khi_robot_control', TestKhiRobotControl)
