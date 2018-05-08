# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for end_effector module"""
import actionlib

import hsrb_interface
from hsrb_interface import _testing as testing
import hsrb_interface.end_effector
import hsrb_interface.exceptions

from nose.tools import assert_raises
from nose.tools import ok_
from std_msgs.msg import Bool
from tmc_control_msgs.msg import GripperApplyEffortAction


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

    def test_gripper_force(self):
        """Test Gripper force action client"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        hsrb_interface.end_effector.Gripper('foo')
        self.action_client_mock.assert_called_with(
            "/hsrb/gripper_controller/apply_force",
            GripperApplyEffortAction)

    def test_gripper_force_fail(self):
        """Test Gripper force(failed to apply force)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result_mock.return_value = True
        action_client_mock.get_state.return_value = \
            actionlib.GoalStatus.PREEMPTED
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.force, -1.0)

    def test_gripper_force_time_out(self):
        """Test Gripper force(timed out)"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = False
        gripper = hsrb_interface.end_effector.Gripper('foo')
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.force, 1.0)

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
