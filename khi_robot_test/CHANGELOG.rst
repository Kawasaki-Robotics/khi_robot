^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package khi_robot_test
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
