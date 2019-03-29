#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':

    #init_node()
    rospy.init_node('message', anonymous=True)
    group = MoveGroupCommander("manipulator")
    exec_vel = 0.5

    while True:

        rospy.loginfo("joint1 start")
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_joint_value_target([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
        group.go()
        rospy.loginfo("joint1 end")

        rospy.loginfo("pose1 start")
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_pose_target([0.5,0.2,0.2,0.0,1.0,0.0])
        group.go()
        rospy.loginfo("pose1 end")

        rospy.loginfo("pose2 start")
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_pose_target([0.5,0.2,0.9,0.0,1.0,0.0])
        group.go()
        rospy.loginfo("pose2 end")

        rospy.loginfo("pose3 start")
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_pose_target([0.5,0.4,0.9,0.0,1.0,0.0])
        group.go()
        rospy.loginfo("pose3 end")

        rospy.loginfo("pose4 start")
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_pose_target([0.5,0.4,0.2,0.0,1.0,0.0])
        group.go()
        rospy.loginfo("pose4 end")