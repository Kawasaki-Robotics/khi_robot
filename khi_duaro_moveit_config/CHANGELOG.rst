^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package khi_duaro_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2021-01-27)
------------------
* Add joint_limits_interface (#31)
  * Add joint_limits_interface
  * Move joint_limits.yaml from moveit_config to description
  * Modify test messages and conditions in test_position_velocity
  * Change duAro's figure length
  * Move tests from khi_robot_control to khi_robot_test
* update duaro body from dcon to fcon (#30)
  OK, I approved.
  Now travis `kinetic` failed due to `ros-kinetic-moveit-ros-visualization`.
  I think it is OK because this PR is not related with the installation and Travis `melodic` passed.
* Contributors: Daisuke NAKAMICHI, Hiroki Matsui

1.1.2 (2019-06-07)
------------------
* add USE_SOURCE_PERMISSIONS in CMakeLists.txt files those package have executable scripts (`#18 <https://github.com/Kawasaki-Robotics/khi_robot/issues/18>`_)
* Contributors: Yosuke Yamamoto

1.1.1 (2019-04-25)
------------------

1.1.0 (2019-04-11)
------------------
* Merge pull request `#10 <https://github.com/Kawasaki-Robotics/khi_robot/issues/10>`_ from d-nakamichi/khi_prefix
  Prefix all pkgs with 'khi\_'
* Convert duaro\_* to khi_duaro\_*
* Contributors: Hiroki Matsui, nakamichi_d

1.0.0 (2019-03-28)
------------------
* Refactoring
* duAro URDF modification
* duAro IKfast
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
