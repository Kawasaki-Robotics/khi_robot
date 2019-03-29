#!/usr/bin/env bash

#Set initial joint positions

sleep 2
rosservice call /gazebo/unpause_physics
sleep 1
#Stop the controllers
rosservice call /controller_manager/switch_controller [] ["duaro_lower_arm_controller","duaro_upper_arm_controller","joint_state_controller"] 2

rosservice call /gazebo/pause_physics
rosservice call /gazebo/set_model_configuration '{ model_name: "robot", urdf_param_name: "robot_description", 
                                                  joint_names: [ "lower_joint1", "lower_joint2", "lower_joint3", "lower_joint4", "upper_joint1", "upper_joint2", "upper_joint3", "upper_joint4" ], 
                                                  joint_positions: [ -0.785, 0.785, 0.0, 0.0, 0.785, -0.785, 0.0, 0.0] }'
sleep 1

#Start the controllers
rosservice call /controller_manager/switch_controller ["duaro_lower_arm_controller","duaro_upper_arm_controller","joint_state_controller"] [] 2 &
sleep 1
rosservice call /gazebo/unpause_physics