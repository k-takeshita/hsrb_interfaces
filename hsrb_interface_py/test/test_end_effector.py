#!/usr/bin/env python3
# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for end_effector module"""
from unittest.mock import patch, PropertyMock

import _testing as testing

from control_msgs.action import FollowJointTrajectory

import hsrb_interface_py
import hsrb_interface_py.end_effector
import hsrb_interface_py.exceptions

from nose.tools import assert_raises
from nose.tools import ok_

from rclpy.duration import Duration
from tmc_control_msgs.action import GripperApplyEffort
from trajectory_msgs.msg import JointTrajectoryPoint

_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07


class EndEffectorTest(testing.RosMockTestCase):

    def setUp(self):
        super(EndEffectorTest, self).setUp()

        patcher = patch("hsrb_interface_py.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface_py.end_effector.Gripper._check_state")
        self.check_state_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.gripper_setting = {
            'topic': 'hoge',
            "joint_names": ["hand_motor_joint"],
            "prefix": "/hsrb/gripper_controller",
            "left_finger_joint_name": "hand_l_spring_proximal_joint",
            "right_finger_joint_name": "hand_r_spring_proximal_joint"
        }
        self.suction_setting = {
            "action": "/suction_control",
            "suction_topic": "/suction_on",
            "pressure_sensor_topic": "/pressure_sensor"
        }

    def test_gripper(self):
        """Test Gripper class"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        ok_(gripper)
        self.action_client_mock.assert_any_call(
            self.connection_mock,
            FollowJointTrajectory,
            "/hsrb/gripper_controller/follow_joint_trajectory")
        self.action_client_mock.assert_any_call(
            self.connection_mock,
            GripperApplyEffort,
            "/hsrb/gripper_controller/grasp")
        self.action_client_mock.assert_any_call(
            self.connection_mock,
            GripperApplyEffort,
            "/hsrb/gripper_controller/apply_force")

    def test_gripper_command(self):
        """Test command"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, True]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        open_angle = 1.2
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        gripper.command(open_angle)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=Duration(seconds=1.0).to_msg()
            )
        ]
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_command_set_motion_time(self):
        """Test command when set motion_time"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, True]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        open_angle = 1.2
        motion_time = 2.0
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        gripper.command(open_angle, motion_time=motion_time)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=Duration(seconds=motion_time).to_msg()
            )
        ]
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_command_sync_false(self):
        """Test command when sync are false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.return_value = False
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        open_angle = 1.2
        gripper.command(open_angle, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=Duration(seconds=1.0).to_msg()
            )
        ]
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false(self):
        """Test apply force when delicate is false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, True]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        effort = 1.0
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        gripper.apply_force(effort)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false_sync_false(self):
        """Test apply force when delicate and sync are false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.return_value = False
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        effort = 1.0
        gripper.apply_force(effort, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_but_equal_to_threshold(self):
        """Test apply force when delicate is true but equal to threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, True]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_and_less_than_threshold(self):
        """Test apply force when delicate is true and less than threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, True]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD - 0.01
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = effort
        action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_effort_negative_value(self):
        """Test apply force(negative effort is set)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        assert_raises(hsrb_interface_py.exceptions.GripperError,
                      gripper.apply_force, -1.0)

    def test_gripper_apply_force_fail(self):
        """Test apply force(failed to apply force)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, False]
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        isdone_mock = PropertyMock(return_value=True)
        type(gripper)._isdone = isdone_mock
        assert_raises(hsrb_interface_py.exceptions.GripperError,
                      gripper.apply_force, 1.0)

    def test_gripper_apply_force_time_out(self):
        """Test apply force(timed out)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        self.check_state_mock.side_effect = [False, False]
        gripper = hsrb_interface_py.end_effector.Gripper('foo')
        assert_raises(hsrb_interface_py.exceptions.GripperError,
                      gripper.apply_force, 1.0)
