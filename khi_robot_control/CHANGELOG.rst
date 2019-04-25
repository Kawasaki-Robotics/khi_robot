^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package khi_robot_control
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2019-04-25)
------------------
* Update a document and error messages (`#15 <https://github.com/Kawasaki-Robotics/khi_robot/issues/15>`_)
  * Modify error messages
  * fixup! Modify error messages
  * fixup! Modify error messages
  * fixup! Modify error messages
  * fixup! Modify error messages
* set HOME position separately (`#16 <https://github.com/Kawasaki-Robotics/khi_robot/issues/16>`_)
* Split command string in two (`#14 <https://github.com/Kawasaki-Robotics/khi_robot/issues/14>`_)
* Merge pull request `#13 <https://github.com/Kawasaki-Robotics/khi_robot/issues/13>`_ from d-nakamichi/load_rtc_param
  Load a temporary RTC param file
* fixup! Load temporary RTC param file
* Merge pull request `#12 <https://github.com/Kawasaki-Robotics/khi_robot/issues/12>`_ from d-nakamichi/control_axes_bug_fix
  Limit AS control axes to ROS driver's control axes
* Load temporary RTC param file
* Limit AS control axes to ROS driver's control axes
* Merge pull request `#11 <https://github.com/Kawasaki-Robotics/khi_robot/issues/11>`_ from d-nakamichi/buildfarm_fix
  Add test_depend
* Add test_depend
* Contributors: Daisuke NAKAMICHI, Hiroki Matsui, nakamichi_d

1.1.0 (2019-04-11)
------------------
* robot_control: fix msg deps of main. (`#9 <https://github.com/Kawasaki-Robotics/khi_robot/issues/9>`_)
  Seems to have been remove in `#6 <https://github.com/Kawasaki-Robotics/khi_robot/issues/6>`_.
* update travis.yml (`#6 <https://github.com/Kawasaki-Robotics/khi_robot/issues/6>`_)
  * update travis.yml
  * rs_desc: add missing install targets.
  * duaro_desc: add missing install targets.
  * rs_gazebo: add missing install targets.
  * duaro_gazebo: add missing install targets.
  * description: pkgs do not run depend on urdf.
  * duaro_desc: add roslaunch testing.
  Tries to load xacro as parameter.
  * rs_desc: add roslaunch testing.
  Tries to load xacros as parameters.
  * robot_control: use imported target for KRNX binary blob.
  Avoid setting link directories.
  * robot_control: warn user if CPU arch could not be detected.
  * robot_control: remove redundant dep declaration.
  'khi_robot_msgs' is already a build_depend and is another package, so build ordering will already take the dependencies into account.
  * robot_control: don't install headers.
  The main library is not exported (by catkin_package(..)), so the headers also don't need to be installed.
  This also resolves an installation issue: the install(..) statement expects a sub dir called 'khi_robot_control' in the 'include' directory, but that doesn't exist.
  * robot_control: sort components alphabetically.
  Make catkin_lint happ(ier).
  * robot_control: fix realtime_tools unconfigured build dep.
  * robot_control: sort source files for robot_client target.
  Make catkin_lint happ(ier).
  * moveit_cfgs: install scripts dir.
  * rs080n_moveit_config does not have script directory
  * set indigo as allow_failures
  * add test for khi_robot_control/main and libkrnx
  * robot_control: work-around for KRNX lib blob linking issues.
  Based on suggestion from @k-okada.
  Not sure whether this is an actual proper solution, or just a work-around.
  Systems with stricter ld library path security might not like this.
* Updates to build scripts and manifests (packaging) (`#5 <https://github.com/Kawasaki-Robotics/khi_robot/issues/5>`_)
  * rs_desc: add missing install targets.
  * duaro_desc: add missing install targets.
  * rs_gazebo: add missing install targets.
  * duaro_gazebo: add missing install targets.
  * description: pkgs do not run depend on urdf.
  * duaro_desc: add roslaunch testing.
  Tries to load xacro as parameter.
  * rs_desc: add roslaunch testing.
  Tries to load xacros as parameters.
  * robot_control: use imported target for KRNX binary blob.
  Avoid setting link directories.
  * robot_control: warn user if CPU arch could not be detected.
  * robot_control: don't install headers.
  The main library is not exported (by catkin_package()), so the headers also don't need to be installed.
  This also resolves an installation issue: the install(..) statement expects a sub dir called 'khi_robot_control' in the 'include' directory, but that doesn't exist.
  * robot_control: sort components alphabetically.
  Make catkin_lint happ(ier).
  * robot_control: fix realtime_tools unconfigured build dep.
  * robot_control: sort source files for robot_client target.
  Make catkin_lint happ(ier).
  * moveit_cfgs: install scripts dir.
  Except RS080N.
  * robot_control: work-around for KRNX lib blob linking issues.
  Based on suggestion from @k-okada.
  Not sure whether this is an actual proper solution, or just a work-around.
  Systems with stricter ld library path security might not like this.
* Contributors: G.A. vd. Hoorn, Kei Okada

1.0.0 (2019-03-28)
------------------
* Refactoring
* Update License
* Modify ERROR/RESTART/QUIT state process
* Modify simulation method
* Add KRNX libraries
* Contributors: nakamichi_d, matsui_hiro

0.9.4 (2019-01-25)
------------------
* modify deactivation in simulation mode
* bug fix of get_status service
* Contributors: nakamichi_d

0.9.3 (2019-01-21)
------------------
* Simple command service
* Bugfix about rs setting files
* Minor coding fix
* Contributors: nakamichi_d, matsui_hiro

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
* Contributors: nakamichi_d, matsui_hiro
