#!/usr/bin/env python

import rospy

from moveit_commander import MoveGroupCommander

#from tork_moveit_tutorial import *

def line_move(name, jbase):
    group = MoveGroupCommander(name)
    exec_vel = 1.0
    diff = 0.1

    print(name + "do line_move")

    pos = group.get_current_pose()
    print("pos")
    print(pos.pose.position)
    group.set_joint_value_target(jbase)
    group.go()

    pos = group.get_current_pose()
    print("pos init")
    print(pos.pose.position)

    # x
    lpos = pos
    lpos.pose.position.x = pos.pose.position.x - diff
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos -x")
    print(lpos.pose.position)
    lpos = pos
    lpos.pose.position.x = pos.pose.position.x + diff
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos +x")
    print(lpos.pose.position)

    # y
    lpos = pos
    lpos.pose.position.y = pos.pose.position.y - diff
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos -y")
    print(lpos.pose.position)
    lpos = pos
    lpos.pose.position.y = pos.pose.position.y + diff
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos +y")
    print(lpos.pose.position)

    # z
    lpos = pos
    lpos.pose.position.z = pos.pose.position.z - diff*0.2
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos -z")
    print(lpos.pose.position)
    lpos = pos
    lpos.pose.position.z = pos.pose.position.z + diff*0.2
    group.set_pose_target(lpos)
    group.go()
    lpos = group.get_current_pose()
    print("pos +z")
    print(lpos.pose.position)

def both_line_move(name, eef1, eef2, jbase):
    group = MoveGroupCommander(name)
    exec_vel = 1.0
    diff = 0.1

    print(name + "do line_move")

    pos1 = group.get_current_pose(eef1)
    print("pos1")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2")
    print(pos2.pose.position)
    group.set_joint_value_target(jbase)
    group.go()

    pos1 = group.get_current_pose(eef1)
    print("pos1 init")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 init")
    print(pos2.pose.position)

    # x
    lpos1 = pos1
    lpos1.pose.position.x = pos1.pose.position.x - diff
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.x = pos2.pose.position.x - diff
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 -x")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 -x")
    print(pos2.pose.position)
    lpos1 = pos1
    lpos1.pose.position.x = pos1.pose.position.x + diff
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.x = pos2.pose.position.x + diff
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 +x")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 +x")
    print(pos2.pose.position)

    # y
    lpos1 = pos1
    lpos1.pose.position.y = pos1.pose.position.y - diff
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.y = pos2.pose.position.y - diff
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 -y")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 -y")
    print(pos2.pose.position)
    lpos1 = pos1
    lpos1.pose.position.y = pos1.pose.position.y + diff
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.y = pos2.pose.position.y + diff
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 +y")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 +y")
    print(pos2.pose.position)

    # z
    lpos1 = pos1
    lpos1.pose.position.z = pos1.pose.position.z - diff*0.2
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.z = pos2.pose.position.z - diff*0.2
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 -z")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 -z")
    print(pos2.pose.position)
    lpos1 = pos1
    lpos1.pose.position.z = pos1.pose.position.z + diff*0.2
    group.set_pose_target(lpos1,eef1)
    lpos2 = pos2
    lpos2.pose.position.z = pos2.pose.position.z + diff*0.2
    group.set_pose_target(lpos2,eef2)
    group.go()
    pos1 = group.get_current_pose(eef1)
    print("pos1 +z")
    print(pos1.pose.position)
    pos2 = group.get_current_pose(eef2)
    print("pos2 +z")
    print(pos2.pose.position)

if __name__ == '__main__':

    rospy.init_node("duaro_line")

    line_move("lower_arm", [-1.6, 1.9, 0.045, 0.0])
    line_move("upper_arm", [1.6, -1.9, 0.045, 0.0])
    both_line_move("botharms", "lower_link_j4", "upper_link_j4", [-1.6, 1.9, 0.045, 0.0, 1.6, -1.9, 0.045, 0.0])