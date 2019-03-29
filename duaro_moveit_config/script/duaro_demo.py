#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':

    #init_node()
    group = MoveGroupCommander("botharms")
    sleep_time = 0.2
    exec_vel = 1.0

    while True:
        # home
        group.set_max_velocity_scaling_factor(exec_vel*0.2)
        group.set_joint_value_target([-0.78, 0.78, 0.14, 0.0, 0.78, -0.78, 0.14, 0.0])
        group.go()
        rospy.sleep(sleep_time)

        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_joint_value_target([-1.5, 1.5, 0.14, 0.0, 1.5, -1.5, 0.14, 0.0])
        group.go()
        rospy.sleep(sleep_time)

        num = 0
        while num < 5:
            group.set_max_velocity_scaling_factor(exec_vel*0.2)
            group.set_joint_value_target([-1.5, 1.5, 0.06, 0.0, 1.5, -1.5, 0.06, 0.0])
            group.go()
            group.set_max_velocity_scaling_factor(exec_vel*0.2)
            group.set_joint_value_target([-1.5, 1.5, 0.14, 0.0, 1.5, -1.5, 0.14, 0.0])
            group.go()
            num += 1

        #group.set_max_velocity_scaling_factor(exec_vel)
        #group.set_joint_value_target([-0.6, 0.78, 0.0, 0.0, 0.6, -0.78, 0.0, 0.0])
        #group.go()
        #rospy.sleep(sleep_time)

        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_joint_value_target([-0.6, 0.78, 0.06, 0.0, 1.5, -1.5, 0.06, 0.0])
        group.go()
        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_joint_value_target([-0.6, 0.78, 0.06, 0.0, 0.6, -0.78, 0.06, 0.0])
        group.go()
        rospy.sleep(sleep_time)

        group.set_max_velocity_scaling_factor(exec_vel)
        group.set_joint_value_target([-0.78, 0.78, 0.06, 0.0, 0.78, -0.78, 0.06, 0.0])
        group.go()
        rospy.sleep(sleep_time)
