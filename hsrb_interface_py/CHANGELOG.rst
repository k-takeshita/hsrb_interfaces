^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hsrb_interface_py
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.2 (2021-04-12)
-------------------
* Update maintainer
* replace tmc_manipulation_msgs to trajectory_msgs
* Update import package from IPython to traitlets
* Contributors: Takafumi Mizuno, satoru_onoda

0.11.1 (2020-05-11)
-------------------
* add lxml dependency
* fix lint
* fix kinetic support
* Contributors: Hiromichi Nakashima

0.11.0 (2018-11-28)
-------------------
* Add gripper.get_distance
* Add stall check
* Add return
* Use CachingSubscriber
* Add gripper.get_distance
* Contributors: 太田 雄介, 寺田 耕志, 木村 哲也, 水野 貴文, 池田 拓也

0.10.0 (2018-05-18)
-------------------
* Add gripper.set_distance
* Integrate grasp and apply_force
* Add gripper.force command
* add tf_client interface
* Use 'if base_trajectory' instead of 'if base_trajectory is not None'
* Add joint_weights property to JointGroup
* initilize once
* Drop first trajectory point in MobileBase.create_follow_trajectory_goal.
* Contributors: Keisuke Takeshita, Yuta Watanabe, koji_terada, yuga_takai, 太田 雄介, 寺田 耕志, 水野 貴文

0.9.0 (2017-10-27)
------------------
* Reset stamp in omni_base.create_follow_trajectory_goal().
* fix method comment
* use base_only fitler for mobile
* add new_timeopt
* Fix document misspelling
* update default tf_timeout
* Use default robot model
* fix dependency
* change tf listener queue_size
* use current time for tf
* Fix Python extenstion library install destination
* Add move_end_effector_by_arc to JointGroup.
* Add async interfaces to MobileBase.
* JointGroup.move_end_effector_pose can receive multi-poses.
* Add gaze_point() to JointGroup.
* Add a follow trajectory interface to MobileBase.
* Add an attribute; looking_hand_constraint
* Add a function to generate a planning service request
* Contributors: Keisuke Takeshita, koji_terada, minori_higuchi, 西野 環

0.8.2 (2017-04-11)
------------------
* add suction command normal test
* add suction command test
* add suction_command input check
* add interfaces to set/get tf timeout
* Check whether planning weights are valid or not
* Check whether planning_timeout is valid or not
* Add interfaces to set/get tf timeout
* JointGroup.move_end_effector_by_line rejects zero vector axis
* JointGroup control hand_motor_joint
* apply pep8
* change_joint_state() reject base_roll_joint
* Contributors: Keisuke Takeshita, takuyaikeda, 寺田 耕志, 水野 貴文

0.8.1 (2016-12-01)
------------------
* Update copyright
* use pep8 style.
* Contributors: Akiyoshi Ochiai, 寺田　耕志, 西野 環

0.8.0 (2016-09-28)
------------------
* Implements move_cartesian_path
* Add unittests
* Move trajectory operations from joint_group.py to trajectory.py
* Update documentation.
* Contributors: Akiyoshi Ochiai, Keisuke Takeshita

0.7.1 (2016-04-13)
------------------
* Fix MobileBase unittest
* Insert SimpleActionClient.wait_for_r_server() to MobileBase() to fix unstable action execution
* Contributors: Akiyoshi Ochiai

0.7.0 (2016-04-12)
------------------
* Write PKGDOC.rst
* Fix error message if impednace_config is invalid
* Remove unnecessary comma
* Add mission wait_for_message to CollisionWorld.add_mesh()
* Fix wrong attribute name in CollisionWorld
* Fix error message if impednace_config is invalid
* Remove raw sensor interfaces from ihsrb
* Remove miscopied comma
* Fix non return bug
* Contributors: Akiyoshi Ochiai, Keisuke Takeshita, 竹下 佳佑

0.6.1 (2016-04-06)
------------------
* Rollback dependencty on the traitlets module to IPython.config.loader
* Contributors: Akiyoshi Ochiai

0.6.0 (2016-04-04)
------------------
* Conform to PEP8 and PEP257
* Change default documentation language to English
* Add base timeopt
* Fix battery timeout
* Add impedance control interface
* Add interfaces to sensors.ForceTorque for handling compensated wrench
* Contributors: Akiyoshi Ochiai, Keisuke Takeshita, Yoshimi Iyoda, 寺田　耕志, 水野 貴文, 落合 亮吉

0.5.2 (2015-11-27)
------------------
* Add patch to urdf_parser to support multiple visuals and collisions
* Contributors: 西野 環

0.5.1 (2015-11-26)
------------------
* Fix empyt id check.
* utilize resource retriver.

0.5.0 (2015-11-24)
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
