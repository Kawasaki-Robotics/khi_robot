^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package khi_robot_bringup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2021-01-27)
------------------
* Merge pull request #36 from d-nakamichi/refactoring
  Refactoring about khi_robot_control
* Refactoring khi_robot_controllers/JointData/KrnxRobotTable/RobotInfo/rtc to khi_robot_param/KhiRobotData/KhiRobotControllerInfo/KhiRobotKrnxRtcData
* Add joint_limits_interface (#31)
  * Add joint_limits_interface
  * Move joint_limits.yaml from moveit_config to description
  * Modify test messages and conditions in test_position_velocity
  * Change duAro's figure length
  * Move tests from khi_robot_control to khi_robot_test
* Add position_controllers/JointGroupPositionController (#22)
  * Add position_controllers/JointGroupPositionController
  * fixup! Add position_controllers/JointGroupPositionController
* Delete README.md in khi_robot_bringup
  This is unneeded.
* Contributors: Daisuke NAKAMICHI, Hiroki Matsui, d-nakamichi

1.1.2 (2019-06-07)
------------------

1.1.1 (2019-04-25)
------------------

1.1.0 (2019-04-11)
------------------
* Merge pull request `#10 <https://github.com/Kawasaki-Robotics/khi_robot/issues/10>`_ from d-nakamichi/khi_prefix
  Prefix all pkgs with 'khi\_'
* Convert rs\_* to khi_rs\_*
* Convert duaro\_* to khi_duaro\_*
* Contributors: Hiroki Matsui, nakamichi_d

1.0.0 (2019-03-28)
------------------
* Refactoring
* duAro IKFast
* Contributors: matsui_hiro, nakamichi_daisuke

0.9.4 (2019-01-25)
------------------

0.9.3 (2019-01-21)
------------------
* GitLab CI
* Bugfix about rs setting files
* Contributors: matsui_hiro, nakamichi_daisuke

0.9.2 (2018-12-27)
------------------
* RESTART function
* KHI Command service
* duAro URDF modification
* RS080N
* Unifying RS series URDF/Gazebo into rs_description, rs_gazebo
* Modification of ACTIVATING state
* Modification of QUIT state
* Modification of state definition
* Changing start position "ALL 0 degree" to "Current degree position"
* Contributors: matsui_hiro, nakamichi_daisuke
