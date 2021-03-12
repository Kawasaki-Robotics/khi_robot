khi_robot [![Build Status](https://travis-ci.com/Kawasaki-Robotics/khi_robot.svg?branch=master)](https://travis-ci.com/Kawasaki-Robotics/khi_robot)
===================================================================================================================================================

このリポジトリではKHIのロボットのROSサポートパッケージを提供しています。  
ROSのディストリビューション`Kinetic`と`Melodic`をサポートしています。

## 起動方法

### 1. 制御ノードの起動

```khi_robot_control``` を以下のように起動してください。:

```
roslaunch khi_robot_bringup ***_bringup.launch ip:=***
```

ロボットを制御せずに閲覧だけしたい場合は、閲覧モード用の引数'viewer'を指定してください。:

```
roslaunch khi_robot_bringup ***_bringup.launch ip:=*** viewer:=true
```

実機のロボットがない場合は、ループバックモード用の引数'simulation'を指定してください。:

```
roslaunch khi_robot_bringup ***_bringup.launch simulation:=true
```

Gazeboシミュレーションを行う場合は、以下のように起動してください。:

```
roslaunch ***_gazebo ***_world.launch
```

### 2. MoveIt!ノードの起動

MoveIt!のスクリプトを以下のように開始してください。:

```
roslaunch ***_moveit_config moveit_planning_execution.launch
```

開始するとMoveIt!のrviz画面が表示されるので、GUIでロボットが操作できるようになります。

## 実機との接続

[docs/ConnectingRealRobot-ja.md](ConnectingRealRobot-ja.md)を参照してください。

## サポートしているロボット

 * duaro
 * rs007l
 * rs007n
 * rs013n
 * rs80n

## ノート

### このソフトウェアについて

このソフトウェアは実験コードです。問題点や機能として不足する点がある場合があります。  
提供するAPIは安定版ではなく、変更する可能性があります。生産システムでの使用は推奨していません。

### 座標系について

KHIとROSの座標系は異なります。

```
KHIの座標系の原点はロボットのLink1です。

ROSの座標系の原点はWorldです。
```

### コントローラについて

`khi_robot_control`はデフォルトで`position_controllers/JointPositionController`を使用しています。`position_controllers/JointGroupPositionController`も使用可能です。  

`position_controllers/JointPositionController` : `***_arm_controller (例)rs007n_arm_controller`  
`position_controllers/JointGroupPositionController` : `***_joint_group_controller (例)rs007n_joint_group_controller`  

利用可能なコントローラはサービス`controller_manager/list_controllers`で確認できます。  
コントローラを切り替えるには、サービス`controller_manager/switch_controller`を使用してください。  

(例)
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

### CADデータについて

`***_ description`はKHIウェブサイトで提供するCADデータを元にしたSTLファイルを使用しています。  
そのため、これらのファイルにも[CADデータ利用上の注意](https://robotics.kawasaki.com/ja1/products/CAD-disclaimer/)が適用されるのでご注意ください。