#!/usr/bin/env python
##########################################
# @file		rectangle.py        	 #
# @brief	rectangle moveing        #
# @author   matsui_hiro				 #
# @date		2018/7/19				 #
##########################################

import moveit_commander
import rospy
import geometry_msgs.msg
import math
import sys
import copy
import moveit_msgs.msg


def main():
	rospy.init_node("moveit_command_sender")

	robot = moveit_commander.RobotCommander()

	print "=" * 10, " Robot Groups:"
	print robot.get_group_names()

	print "=" * 10, " Printing robot state"
	print robot.get_current_state()
	print "=" * 10 

	manipulator = moveit_commander.MoveGroupCommander("manipulator")
	print "=" * 15, " manipulator ", "=" * 15
	print "=" * 10, " Reference frame: %s" % manipulator.get_planning_frame()
	print "=" * 10, " Reference frame: %s" % manipulator.get_end_effector_link()

	#set planner
	manipulator.set_planner_id("RRTkConfigDefault")

	#set parameter
	accuracy_posi = 0.001#position accuracy
	accuracy_ori = 0.001 #orientation accuracy
	cub_link = 0.32      #cube length
	num_cyc = 1          #cycle unmber
	exec_vel = 0.98      #max velocity
	exec_accel = 1       #max acceleration

	manipulator.set_max_velocity_scaling_factor(exec_vel)
	manipulator.set_max_acceleration_scaling_factor(exec_accel)
	manipulator.set_goal_orientation_tolerance(accuracy_ori)
	manipulator.set_goal_position_tolerance(accuracy_posi)

	# arm Initial Pose
	manipulator_initial_pose = manipulator.get_current_pose().pose
	print "=" * 10, " Printing Right Hand initial pose: "
	print manipulator_initial_pose
	manipulator.set_joint_value_target([-22.2/180.0*math.pi, 48.1/180.0*math.pi, -121.5/180.0*math.pi, 0.5/180.0*math.pi, -10.5/180.0*math.pi, -90.0/180.0*math.pi])
	manipulator.go()
	# rospy.sleep(30)
	# manipulator.set_joint_value_target([-12.2/180.0*math.pi, 48.1/180.0*math.pi, -121.5/180.0*math.pi, 0.5/180.0*math.pi, -10.5/180.0*math.pi, -90.0/180.0*math.pi])
	# manipulator.go()
	# rospy.sleep(30)
	# manipulator.set_joint_value_target([0.0/180.0*math.pi, 38.1/180.0*math.pi, -121.5/180.0*math.pi, 0.5/180.0*math.pi, -10.5/180.0*math.pi, -90.0/180.0*math.pi])
	# manipulator.go()
	# rospy.sleep(30)

	rospy.sleep(3)

	waypoints = []

	# start with the current pose
	waypoints.append(manipulator.get_current_pose().pose)
	waypoints[0].position.y += cub_link
	waypoints[0].position.z += cub_link

	# first orient gripper and move forward (+x)
	wpose = geometry_msgs.msg.Pose()
	wpose.position.x = waypoints[0].position.x + cub_link
	wpose.position.y = waypoints[0].position.y
	wpose.position.z = waypoints[0].position.z
	wpose.orientation.x = waypoints[0].orientation.x
	wpose.orientation.y = waypoints[0].orientation.y
	wpose.orientation.z = waypoints[0].orientation.z
	wpose.orientation.w = waypoints[0].orientation.w
    
	waypoints.append(copy.deepcopy(wpose))

	# second move down
	wpose.position.y -= cub_link
	wpose.position.z -= cub_link
	waypoints.append(copy.deepcopy(wpose))

	# third move to the side
	wpose.position.x -= cub_link 
	waypoints.append(copy.deepcopy(wpose))
	(plan, fraction) = manipulator.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.1,         # eef_step
                             0.0)         # jump_threshold

	print "============ Waiting while RVIZ displays plan..."
	print waypoints

	# rectangle move start
	num = 0
	while num < num_cyc:
		manipulator.execute(plan)
		num = num + 1




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
