khi_robot [![Build Status](https://travis-ci.com/Kawasaki-Robotics/khi_robot.svg?branch=master)](https://travis-ci.com/Kawasaki-Robotics/khi_robot)
===================================================================================================================================================

This repository provides ROS support for KHI robots.  
ROS distribution `Kinetic` and `Melodic` are supported.

## How to Launch

### 1. Launch Control Node

Start ```khi_robot_control``` as:

```
roslaunch khi_robot_bringup ***_bringup.launch ip:=***
```

If you only want to view robot(not control), specify the argument 'viewer' to use viewer mode:

```
roslaunch khi_robot_bringup ***_bringup.launch ip:=*** viewer:=true
```

If you have no real robot, specify the argument 'simulation' to use loopback mode:

```
roslaunch khi_robot_bringup ***_bringup.launch simulation:=true
```

If you want to use gazebo simulation:

```
roslaunch ***_gazebo ***_world.launch
```

### 2. Launch MoveIt! Node

Start a MoveIt! script as:

```
roslaunch ***_moveit_config moveit_planning_execution.launch
```

Now you can see the rviz screen of MoveIt! and interact the robot with the GUI.

## Connecting Real Robot

Refer to [docs/ConnectingRealRobot.md](docs/ConnectingRealRobot.md)

## Supported Robot

 * duaro
 * rs007l
 * rs007n
 * rs013n
 * rs80n

## Notes

### About this software

This software is experimental code. There are known issues and missing functionality.  
The APIs are completely unstable and likely to change. Use in production systems is not recommended.

### About Coordinate

KHI coordinate and ROS cordinate are different.

```
Origin of KHI coordinate is Robot Link1 origin.

Origin of ROS coordinate is World origin.
```

### About controllers

`khi_robot_control` uses `position_controllers/JointPositionController` as default, and it can also use `position_controllers/JointGroupPositionController`.  

`position_controllers/JointPositionController` : `***_arm_controller (e.g.)rs007n_arm_controller`  
`position_controllers/JointGroupPositionController` : `***_joint_group_controller (e.g.)rs007n_joint_group_controller`  

To check available controllers, you can use service `controller_manager/list_controllers`.  
To switch controllers, you can use service `controller_manager/switch_controller`.  

(e.g.)
```
$ rosservice call /controller_manager/list_controllers
controller: 
  - 
    name: "rs007n_joint_group_controller"
    state: "stopped"
    type: "position_controllers/JointGroupPositionController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::PositionJointInterface"
        resources: [joint1, joint2, joint3, joint4, joint5, joint6]
  - 
    name: "joint_state_controller"
    state: "running"
    type: "joint_state_controller/JointStateController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::JointStateInterface"
        resources: []
  - 
    name: "rs007n_arm_controller"
    state: "running"
    type: "position_controllers/JointTrajectoryController"
    claimed_resources: 
      - 
        hardware_interface: "hardware_interface::PositionJointInterface"
        resources: [joint1, joint2, joint3, joint4, joint5, joint6]
```
```
$ rosservice call /controller_manager/switch_controller "start_controllers:
- 'rs007n_joint_group_controller'
stop_controllers:
- 'rs007n_arm_controller'
strictness: 2" 
ok: True
```
(http://wiki.ros.org/controller_manager)  

### About CAD data

`***_ description` are using STL files based on CAD Data of the KHI website.  
Therefore [KHI CAD Data Disclaimer](https://robotics.kawasaki.com/en1/products/CAD-disclaimer/?language_id=1) is also applied to these files.

## Other Languages

[日本語](docs/README-ja.md)