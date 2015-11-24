^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrb_interface_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
------------------
* Fix #54
* Support removing collision object by ID
* Add change base IK weight property
* Add collision checking
* Enable Ctrl-C in interactive mode
* Add joint limits property
* Display human readable error codes.
* More friendly error message in WholeBody._change_joint_state()

0.4.0 (2015-08-20)
------------------
* Update documentation
* Add keyword arguments to move_to_joint_position
* Avoid importing math and geometry
* Add compatibility fuction in geometry.py
* Import utilities directly in ihsrb
* Add shebang line
* Change try_get timeout to 1.0
* Add image attributes
* Add geometry constructors
* Fix #28
* Fix #27
* Fix #31
* Fix get pose method
* Contributors: Akiyoshi Ochiai

0.3.0 (2015-07-30)
------------------
* Fix test error
* Fix Image.to_cv()
* Fix #25
* Add try_get method to Robot class
* Contributors: Akiyoshi Ochiai

0.2.0 (2015-07-29)
------------------
* Add object.get_pose
* Fix #19
* Apply pyflakes
* Fix #23
* Implement #7
* Fix #6
* Fix #10
* Add HSR-B Interactive Shell (ihsrb)
* Fix #20
* Fix #18
* Fix #14
* Fix #11
* Install hsrb_operator.py
* Fix gripper command
* Fix #8
* change method name.
* Update reference manual
* add initial pose for autonmous movement.
* fix end_effector bug.
  - change arg name distance to angle.
  - add keyword arg of time.
* Merge branch 'feature/fix_minor_bugs' of /var/git/repositories/hsr/hsrb_interfaces into develop
* change map name
* add ipython script
* Split MobileBase interface
* fix minor bugs
* Make goto method generic
* add goto_pose test.
* add description.
* add pose move interface.
* Contributors: Akiyoshi Ochiai, 寺田　耕志, 落合　亮吉, 西野 環

0.1.0 (2015-07-20)
------------------
* change pose topic
* Fix package descriptions
* Fix build errors
* Add ItemTypes enum
* Add object_detection test
* Add mobile_base test
* Add missing dependencies
* Add tf to run_depend
* Remove unused build_depends
* Add queue_size argument to suction publisher
* Add text_to_speech tests
* Remove unnecessary print statement
* Apply catkin_lint
* Merge branch 'develop' of git.probo:hsr/hsrb_interfaces into develop
* Update
* fix target_pose.header.stamp
* support install_requires
* fix member name.
* Rename object_detector.py to object_detection.py
* fix base rotation angle.
* Add test stubs
* Make internal function protected
* set non-planned joint velocity and acceleration to 0.0
* Merge branch 'develop' of git.probo:hsr/hsrb_interfaces into develop
* Fix joint_group errors
* change rate for cheking trajectory action result.
* Remove run_depend
* Update resource management system
* Merge branch 'develop' of git.probo:hsr/hsrb_interfaces into develop
* Add more tests
* Add tests for sensors, battery, utils
* Update resource management system
* Add test files
* Update API reference
* Implement resource management
* Implement resource management
* Add move_hand_by_line
* Add minimum sphinx doc files
* Initial commit
* Contributors: Akiyoshi Ochiai, 寺田　耕志, 西野 環
