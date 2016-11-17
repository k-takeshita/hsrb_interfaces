#!/usr/bin/env python
# vim: fileencoding=utf-8 :
# Copyright (C) 2016 Toyota Motor Corporation
"""Testing motion planning interface in Gazebo simulator."""

import math

from hsrb_interface import geometry
import testing


class WholeBodyTest(testing.HsrbInterfaceTest):
    """Test cases for whole_body object."""

    def test_move_to_named_positions(self):
        """Test fixed pattern pose changes."""
        self.whole_body.move_to_neutral()
        self.expect_joints_reach_goals(self.EXPECTED_NEUTRAL, 0.01)

        self.whole_body.move_to_go()
        self.expect_joints_reach_goals(self.EXPECTED_TO_GO, 0.01)

        self.whole_body.move_to_neutral()
        self.expect_joints_reach_goals(self.EXPECTED_NEUTRAL, 0.01)

    def test_move_to_joint_positions(self):
        """Driving each joint."""
        self.whole_body.move_to_neutral()

        self.assertListEqual(self.JOINT_NAMES, self.whole_body.joint_names)

        self.whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        expected_pose = {'arm_lift_joint': 0.2}
        self.expect_joints_reach_goals(expected_pose, delta=0.01)

        self.whole_body.move_to_joint_positions({'head_pan_joint': 0.4,
                                                 'head_tilt_joint': -0.2})
        expected_pose = {'head_pan_joint': 0.4,
                         'head_tilt_joint': -0.2}
        self.expect_joints_reach_goals(expected_pose, delta=0.01)

    def stest_move_end_effector_pose(self):
        """The robot should move end-effector to given pose."""
        self.whole_body.move_to_neutral()
        self.expect_joints_reach_goals(self.EXPECTED_NEUTRAL, delta=0.01)

        hand_pose = self.whole_body.get_end_effector_pose('map')
        self.whole_body.move_end_effector_pose(geometry.pose(z=1.0),
                                               'hand_palm_link')
        goal = ((hand_pose[0].x + 1.0, hand_pose[0].y, hand_pose[0].z),
                hand_pose[1])
        self.expect_hand_reach_goal(goal, pos_delta=0.02,
                                    ori_delta=math.radians(2.0))

    def test_move_end_effector_by_line(self):
        """The robot should move end-effector by line."""
        self.whole_body.move_to_neutral()
        hand_pose = self.whole_body.get_end_effector_pose('map')

        self.whole_body.move_end_effector_by_line((0, 0, 1), 0.1,
                                                  'hand_palm_link')

        goal = ((hand_pose[0].x + 0.1, hand_pose[0].y, hand_pose[0].z),
                hand_pose[1])

        self.expect_hand_reach_goal(goal, frame='map', pos_delta=0.02,
                                    ori_delta=math.radians(2.0))

    def test_move_hand_pose_with_tf(self):
        """Moving end-effector with a tf frame 'my_frame'."""
        self.whole_body.move_to_neutral()

        goals = [
            geometry.pose(),
            geometry.pose(x=0.3),
            geometry.pose(x=0.2, ej=-1.57)
        ]

        for goal in goals:
            self.whole_body.move_end_effector_pose(goal, 'my_frame')
            self.expect_hand_reach_goal(goal, pos_delta=0.02,
                                        ori_delta=math.radians(2.0),
                                        frame='my_frame')

    def test_move_cartesian_path(self):
        """Moving end-effector with following specified waypoints"""
        self.whole_body.move_to_neutral()
        waypoints = [
            geometry.pose(0.0, 0.0, 0.7, ej=-math.pi / 2.0, ek=-math.pi),
            geometry.pose(0.5, 0.5, 0.7),
            geometry.pose(1.0, 1.0, 0.7, ej=-math.pi / 2.0, ek=math.pi),
            geometry.pose(0.5, 0.5, 0.7, ei=math.pi)]
        self.whole_body.move_cartesian_path(waypoints, ref_frame_id='odom')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_whole_body', WholeBodyTest)
