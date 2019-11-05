#!/usr/bin/env python
# coding:utf-8
##########################################
# @file		position_velocity_test.py
# @brief	each joint move
# @author	matsui_hiro, d-nakamichi
# @date		2019/7/9
##########################################

PKG = 'khi_robot_control'
import roslib; roslib.load_manifest(PKG)

import copy
import math
import sys
import unittest
import rospy
import moveit_commander
import geometry_msgs.msg
from khi_robot_msgs.srv import *

service = 'khi_robot_command_service'
sender = 'moveit_command_sender'
planner = 'RRTConnectkConfigDefault'

class KhiRobot:
    arm_name = ''
    arm_num = 1
    max_jt = 6
    group = 'manipulator'
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
        self.arm_name = rospy.get_param('/khi_robot_controllers/robot')
        limits = rospy.get_param('/'+self.arm_name+'/joint_limits')
        if self.arm_name == 'WD002N':
            self.arm_num = 2
            self.max_jt = 8
            self.group = 'botharms'
            self.base_pos_list = [ -90*math.pi/180, 0, 0, 0, 90*math.pi/180, 0, 0, 0 ]
            for jt in range(len(limits)):
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
            for jt in range(len(limits)):
                self.min_pos_list.append(limits['joint'+str(jt+1)]['min_position'])
                self.max_pos_list.append(limits['joint'+str(jt+1)]['max_position'])
                self.max_vel_list.append(limits['joint'+str(jt+1)]['max_velocity'])
                self.max_acc_list.append(limits['joint'+str(jt+1)]['max_acceleration'])
        print('base_position:',self.base_pos_list)
        print('min_position:',self.min_pos_list)
        print('max_position:',self.max_pos_list)
        print('max_velocity:',self.max_vel_list)
        print('max_acceleration:',self.max_acc_list)

    def get_pos_list(self, ano, jt, type):
        jt_list = copy.deepcopy(self.base_pos_list)
        if self.arm_name.startswith('RS'):
            if type == 'min':
                if jt+1 == 2:
                    jt_list[2] = -130*math.pi/180
                jt_list[jt] = self.min_pos_list[jt]
            else:
                if jt+1 == 2:
                    jt_list[2] = 130*math.pi/180
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
    except rospy.ServiceException, e:
        print('Service call failed: %s'%e)

def get_driver_state():
    ret = cmdhandler_client('driver' , 'get_status')
    return ret

def plan_and_execute(mgc,jt,num,timeout):
    for cnt in range(timeout):
        plan = mgc.plan()
        if plan is False:
            print('JT%d-%d: cannot be planned' % (jt+1,num+1))
        else:
            ret = mgc.execute(plan)
            if ret is False:
                print('JT%d-%d: cannot be executed' % (jt+1,num+1))
            else:
                return True

        if cnt == timeout-1:
            print('timeout')
            return False
    return False

class TestKhiRobotControl(unittest.TestCase):

    def test_position_velocity(self):
        ##############################set parameter############################
        accuracy_pos = 0.001    # position accuracy
        accuracy_ori = 0.001    # orientation accuracy

        max_vel = 1.0           # max velocity scale
        max_acc = 1.0           # max acceleration scale
        min_vel = 0.5           # min velocity scale
        min_acc = 0.5           # min acceleration scale

        cyc_num = 3             # reperat num
        timeout = 3             # timeout num
        retcode = 0
        ########################################################################

        ######################################### set range of move #####################################################
        rospy.init_node(sender)
        khi_robot = KhiRobot()

        # RobotCommander
        rc = moveit_commander.RobotCommander()
        print('=' * 15, ' robot ', '=' * 15)
        print('=' * 10, ' Robot Groups: %s' % rc.get_group_names())
        print('=' * 10, ' Printing robot state %s' % rc.get_current_state())

        # MoveGroupCommander
        mgc = moveit_commander.MoveGroupCommander(khi_robot.group)
        print('=' * 15, ' manipulator ', '=' * 15)
        print('=' * 10, ' Reference frame: %s' % mgc.get_planning_frame())
        print('=' * 10, ' Reference frame: %s' % mgc.get_end_effector_link())

        # mgc setting
        mgc.set_planner_id(planner)
        mgc.set_goal_position_tolerance(accuracy_pos)
        mgc.set_goal_position_tolerance(accuracy_ori)

        ret = get_driver_state()
        if ret.cmd_ret == 'ERROR':
            cmdhandler_client('driver', 'restart')
            rospy.sleep(3)

        ret = get_driver_state()
        self.assertEqual('ACTIVE', ret.cmd_ret)

        print('=' * 15, ' test start ', '=' * 15)

        # move each joint
        for jt in range(khi_robot.max_jt):
            if khi_robot.arm_name == 'WD002N':
                if jt < 4:
                    ano = 0
                else:
                    ano = 1
            else:
                ano = 0
    
            fname = 'r%d_maxsp_jt%s_v%d_a%d' % (ano+1,jt+1,max_vel*100, max_acc*100)
    
            # min position/velocity/acceleration
            mgc.set_max_velocity_scaling_factor(min_vel)
            mgc.set_max_acceleration_scaling_factor(min_acc)
            jt_list = khi_robot.get_pos_list(ano, jt, 'min')
            print('JT%d  : base' % (jt+1),jt_list)
            mgc.set_joint_value_target(jt_list)
            self.assertTrue(plan_and_execute(mgc,jt,-1,timeout))

            # max velocity/acceleration
            mgc.set_max_velocity_scaling_factor(max_vel)
            mgc.set_max_acceleration_scaling_factor(max_acc)
    
            # do repeat moving
            for num in range(cyc_num):
                # max position
                jt_list = khi_robot.get_pos_list(ano, jt, 'max')
                print('JT%d-%d:  max ' % (jt+1,num+1),jt_list)
                mgc.set_joint_value_target(jt_list)
                retcode = plan_and_execute(mgc,jt,num,timeout)
                self.assertTrue(retcode)
                if retcode == False:
                    break
                now_jt_list = mgc.get_current_joint_values()
                self.assertAlmostEqual(jt_list[jt], now_jt_list[jt], 3)

                # min position
                jt_list = khi_robot.get_pos_list(ano, jt, 'min')
                print('JT%d-%d:  min' % (jt+1,num+1),jt_list)
                mgc.set_joint_value_target(jt_list)
                retcode = plan_and_execute(mgc,jt,1,timeout)
                self.assertTrue(retcode)
                if retcode == False:
                    break
                now_jt_list = mgc.get_current_joint_values()
                self.assertAlmostEqual(jt_list[jt], now_jt_list[jt], 3)

                # state check
                ret = get_driver_state()
                self.assertEqual('ACTIVE', ret.cmd_ret)
                if ret.cmd_ret != 'ACTIVE':
                    print('=' * 10,'[JT%d][v:%d][a:%d]now driver stats::%s' % (jt+1,max_vel*100,max_acc*100,ret.cmd_ret))
    
            # finish repeat moveing
            ret = get_driver_state()
            self.assertEqual('ACTIVE', ret.cmd_ret)
            if ret.cmd_ret != 'ACTIVE':
                break

        print('=' * 15, ' test end : ', retcode, '=' * 15)
        cmdhandler_client('driver' , 'quit')

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_khi_robot_control', TestKhiRobotControl)