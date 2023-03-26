#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':

    #init_node()
    rospy.init_node('message', anonymous=True)
    group = MoveGroupCommander("manipulator")
    exec_vel = 0.5
    joint1 = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    pose1 = [0.5,0.2,0.2,0.0,1.0,0.0]
    pose2 = [0.5,0.2,0.9,0.0,1.0,0.0]
    pose3 = [0.5,0.4,0.9,0.0,1.0,0.0]
    pose4 = [0.5,0.4,0.2,0.0,1.0,0.0]

    print '>>'
    raw_input()		
    print joint1
    rospy.loginfo("joint1 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_joint_value_target(joint1)
    group.go()
    rospy.loginfo("joint1 end")
    print '>>'
    raw_input()		
    print pose1
    rospy.loginfo("pose1 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_pose_target(pose1)
    group.go()
    rospy.loginfo("pose1 end")
    print '>>'
    raw_input()		
    print pose2
    rospy.loginfo("pose2 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_pose_target(pose2)
    group.go()
    rospy.loginfo("pose2 end")
    print '>>'
    raw_input()		
    print pose3
    rospy.loginfo("pose3 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_pose_target(pose3)
    group.go()
    rospy.loginfo("pose3 end")
    print '>>'
    raw_input()		
    print pose4
    rospy.loginfo("pose4 start")
    group.set_max_velocity_scaling_factor(exec_vel)
    group.set_pose_target(pose4)
    group.go()
    rospy.loginfo("pose4 end")