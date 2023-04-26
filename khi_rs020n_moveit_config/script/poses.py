#!/usr/bin/env python

import tf
import sys
import math
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander import MoveGroupCommander
from moveit_commander.conversions import pose_to_list

LEN_LINK0 = 0.505

def main():
	try:
		rospy.init_node('message', anonymous=True)
		exec_vel = 0.05
		robot = moveit_commander.RobotCommander()
		group = moveit_commander.MoveGroupCommander("manipulator")

		cyc01x = [100.0, 468.25, -50.0, -0.0, 180.0, 0.0]
		cyc02x = [-100.0, 468.25, 0.0,  -0.0, 180.0, 0.0]
		cyc03x = [100.0, 468.25, 0.0,  -0.0, 180.0, 0.0]
		cyc04x = [-100.0, 468.25, -50.0,  -0.0, 180.0, 0.0]
		
		print '>>'
		raw_input()
		print cyc01x
		group.set_pose_target(get_wpose(cyc01x))
		group.go()
		print '>>'
		raw_input()		
		print cyc02x
		group.set_pose_target(get_wpose(cyc02x))
		group.go()
		print '>>'
		raw_input()		
		print cyc03x
		group.set_pose_target(get_wpose(cyc03x))
		group.go()
		print '>>'
		raw_input()		
		print cyc04x
		group.set_pose_target(get_wpose(cyc04x))
		group.go()
		print '>>'
		raw_input()		
		# waypoints = []
		# waypoints.append(get_wpose(cyc03x))
		# waypoints.append(get_wpose(cyc02x))
		# waypoints.append(get_wpose(cyc03x))
		# waypoints.append(get_wpose(cyc04x))
		# print type(waypoints)
		# print waypoints
		# group.set_pose_targets(waypoints)
		# plan = group.plan()

		# print '>>'
		# raw_input()
		# group.execute(plan)

	except rospy.ROSInterruptException:
		return

	except KeyboardInterrupt:
		return

def get_wpose(pos):

	pos[0] = pos[0]/1000.0
	pos[1] = pos[1]/1000.0
	pos[2] = pos[2]/1000.0 + LEN_LINK0
	pos[3] = pos[3]/180.0*math.pi
	pos[4] = pos[4]/180.0*math.pi
	pos[5] = pos[5]/180.0*math.pi

	q = tf.transformations.quaternion_from_euler(pos[3], pos[4], pos[5], 'rzyz')

	wpose = geometry_msgs.msg.Pose()

	wpose.position.x = pos[0]
	wpose.position.y = pos[1]
	wpose.position.z = pos[2]
	wpose.orientation.x = q[0]
	wpose.orientation.y = q[1]
	wpose.orientation.z = q[2]
	wpose.orientation.w = q[3]

	print wpose
	return wpose

if __name__ == '__main__':
   	main()

