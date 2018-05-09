# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for end_effector module"""
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction
import hsrb_interface
from hsrb_interface import _testing as testing
import hsrb_interface.end_effector
import hsrb_interface.exceptions

from nose.tools import assert_raises
from nose.tools import ok_
from std_msgs.msg import Bool
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal

_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07


class EndEffectorTest(testing.RosMockTestCase):

    def setUp(self):
        super(EndEffectorTest, self).setUp()
        self.gripper_setting = {
            'topic': 'hoge',
            "joint_names": ["hand_motor_joint"],
            "prefix": "/hsrb/gripper_controller"
        }
        self.suction_setting = {
            "action": "/suction_control",
            "suction_topic": "/suction_on",
            "pressure_sensor_topic": "/pressure_sensor"
        }

    def test_gripper(self):
        """Test Gripper class"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        gripper = hsrb_interface.end_effector.Gripper('foo')
        ok_(gripper)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/grasp",
            GripperApplyEffortAction)
        self.action_client_mock.assert_any_call(
            "/hsrb/gripper_controller/apply_force",
            GripperApplyEffortAction)

    def test_gripper_apply_force_delicate_false(self):
        """Test Gripper apply force when delicate is false"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = 1.0
        try:
            gripper.apply_force(effort)
        except Exception:
            self.fail()
        expected_gaol = GripperApplyEffortGoal()
        expected_gaol.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal.assert_called_with(expected_gaol)

    def test_gripper_apply_force_delicate_true_but_equal_to_threshold(self):
        """Test Gripper apply force when delicate is true but equal to threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD
        try:
            gripper.apply_force(effort, delicate=True)
        except Exception:
            self.fail()
        expected_gaol = GripperApplyEffortGoal()
        expected_gaol.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        action_client_mock.send_goal.assert_called_with(expected_gaol)

    def test_gripper_apply_force_delicate_true_and_less_than_threshold(self):
        """Test Gripper apply force when delicate is true and less than threshold"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD - 0.01
        try:
            gripper.apply_force(effort, delicate=True)
        except Exception:
            self.fail()
        expected_gaol = GripperApplyEffortGoal()
        expected_gaol.effort = effort
        action_client_mock.send_goal.assert_called_with(expected_gaol)

    def test_gripper_apply_force_effort_negative_value(self):
        """Test Gripper force(negative effort is set)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, -1.0)

    def test_gripper_apply_force_fail(self):
        """Test Gripper force(failed to apply force)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.PREEMPTED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, 1.0)

    def test_gripper_apply_force_time_out(self):
        """Test Gripper force(timed out)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = False
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, 1.0)

    def test_suction(self):
        """Test Suction class"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        ok_(suction)
        self.get_entry_mock.assert_called_with('end_effector', 'foo')

    def test_suction_command(self):
        """Test Suction command(exceptions are never thrown)"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        suction.command(True)
        suction.command(False)
        self.publisher_mock.assert_called_with("/suction_on",
                                               Bool,
                                               queue_size=0)

    def test_suction_command_negative_num(self):
        """Test Suction command(negative number)"""
        self.get_entry_mock.side_effect = [self.suction_setting]
        suction = hsrb_interface.end_effector.Suction('foo')
        assert_raises(ValueError, suction.command, -1)
