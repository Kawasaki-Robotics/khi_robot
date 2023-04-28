^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package khi_rs_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.4.0 (2023-04-28)
------------------
* Merge pull request `#79 <https://github.com/Kawasaki-Robotics/khi_robot/issues/79>`_ from ohno-atsushi/add_rs020n
  Add rs020n
* Add correction rs020n
* Add rs020n
* Merge pull request `#72 <https://github.com/Kawasaki-Robotics/khi_robot/issues/72>`_ from matsui-hiro/rs025n_realrobot
  Add rs025n for real robot
* Change joint_limits.yml
* Merge pull request `#68 <https://github.com/Kawasaki-Robotics/khi_robot/issues/68>`_ from matsui-hiro/add_rs025n
  Add rs025n
* Rename roslaunch_test_rs025n .xml to roslaunch_test_rs025n.xml
* Add rs025n
* Contributors: Hiroki Matsui, HirokiTakami, matsui-hiro, matsui_hiro, ohno_atsushi

1.3.0 (2022-04-05)
------------------
* Add rs013n (`#50 <https://github.com/Kawasaki-Robotics/khi_robot/issues/50>`_)
  * Add rs013n
  * Update rs013n pkgs
  * Update libkrnx
  * Fixed rs013n_bringup.launch to use xacro
  * Update libkrnx to 2.2.0
  * Modify test program for RS series robots
  * Remove kinetic from travis.yml
* Bugfix for noetic release (`#44 <https://github.com/Kawasaki-Robotics/khi_robot/issues/44>`_)
  * update .travis.yml to add noetic test
  * update .travis.yml to change ubuntu version for noetic test
  * Changed to use xacro instead of xacro.py which is deprecated. like https://github.com/ros/urdf_sim_tutorial/pull/8
  * fixed "except A, a:" to "except A as a:" for python3 compatibility
  * add version checking of moveit_commander to the test script due to changes of the return value of MoveGroupCommander.plan() in 1.1.0
  * remove armhf build test for ubuntu 20.04
* Contributors: Hiroki Matsui, Koki Shinjo

1.2.0 (2021-01-27)
------------------
* Merge branch 'master' into refactoring
* add missing config directory in install(DIRECTORY (#34)
  * add missing config directory in install(DIRECTORY
  * add missing config directory in khi_rs_description
  Co-authored-by: Daisuke NAKAMICHI <44663870+d-nakamichi@users.noreply.github.com>
* Add joint_limits_interface (#31)
  * Add joint_limits_interface
  * Move joint_limits.yaml from moveit_config to description
  * Modify test messages and conditions in test_position_velocity
  * Change duAro's figure length
  * Move tests from khi_robot_control to khi_robot_test
* Contributors: Daisuke NAKAMICHI, Kei Okada, d-nakamichi

1.1.2 (2019-06-07)
------------------
* add KHI CAD Data Disclaimer to README (`#20 <https://github.com/Kawasaki-Robotics/khi_robot/issues/20>`_)
  * add KHI CAD Data Disclaimer to README
  * add KHI CAD Data Disclaimer to README_2
  * add KHI license tag to package.xml
  * modify CAD data message of README
* Contributors: Hiroki Matsui

1.1.1 (2019-04-25)
------------------

1.1.0 (2019-04-11)
------------------
* Merge pull request `#10 <https://github.com/Kawasaki-Robotics/khi_robot/issues/10>`_ from d-nakamichi/khi_prefix
  Prefix all pkgs with 'khi\_'
* Convert rs\_* to khi_rs\_*
* Contributors: Hiroki Matsui, nakamichi_d

1.0.0 (2019-03-28)
------------------
* Refactoring
* Contributors: matsui_hiro, nakamichi_d

0.9.4 (2019-01-25)
------------------

0.9.3 (2019-01-21)
------------------

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
* Contributors: matsui_hiro, nakamichi_d
