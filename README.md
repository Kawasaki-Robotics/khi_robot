# khi_robot Package

This repository provides ROS support for KHI robots.  
ROS distribution `Kinetic` is supported.

## How to Launch

### 1. Launch Control Node

Start khi_robot_control as:

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
roslaunch ***_moveit_config ***_moveit.launch
```

Now you can see the rviz screen of MoveIt! and interact the robot with the GUI.

## Connecting Real Robot

Refer to `docs/ConnectingRealRobot.md`

## Supported Robot

 * duaro (currently simulation only)
 * rs007l
 * rs007n
 * rs80n

## Notes

### About this software

This software is experimental code. There are known issues and missing functionality.  
The APIs are completely unstable and likely to change. Use in production systems is not recommended.

### Coordinate

KHI coordinate and ROS cordinate are different.

```
Origin of KHI coordinate is Robot Link1 origin.

Origin of ROS coordinate is World origin.
```