# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for end_effector module"""
import rclpy
from hsrb_interface import Robot

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
import hsrb_interface
from hsrb_interface import _testing as testing
import hsrb_interface.end_effector
import hsrb_interface.exceptions


from mock import ANY
from nose.tools import assert_raises
from nose.tools import ok_
import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tmc_control_msgs.action import GripperApplyEffort
from trajectory_msgs.msg import JointTrajectoryPoint

_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07


class EndEffectorTest(testing.RosMockTestCase):

    def setUp(self):
        super(EndEffectorTest, self).setUp()
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
        rclpy.init()
        robot = Robot()
        """Test Gripper class"""
        self.get_entry_mock.side_effect = [self.gripper_setting]
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        ok_(gripper)

    def test_gripper_command(self):
        """Test command"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        open_angle = 1.2
        gripper.command(open_angle)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg())
        ]
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_command_set_motion_time(self):
        """Test command when set motion_time"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        open_angle = 1.2
        motion_time = 2.0
        gripper.command(open_angle, motion_time=motion_time)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=rclpy.duration.Duration(seconds=motion_time).to_msg()
            )
        ]
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_command_sync_false(self):
        """Test command when sync are false"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        open_angle = 1.2
        gripper.command(open_angle, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = FollowJointTrajectory.Goal()
        expected_goal.trajectory.joint_names = ["hand_motor_joint"]
        expected_goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg())
        ]
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false(self):
        """Test apply force when delicate is false"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        effort = 1.0
        gripper.apply_force(effort)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_false_sync_false(self):
        """Test apply force when delicate and sync are false"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        effort = 1.0
        gripper.apply_force(effort, sync=False)
        self.assertFalse(action_client_mock.wait_for_result.called)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_but_equal_to_threshold(self):
        """Test apply force when delicate is true but equal to threshold"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_delicate_true_and_less_than_threshold(self):
        """Test apply force when delicate is true and less than threshold"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        effort = _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD - 0.01
        gripper.apply_force(effort, delicate=True)
        expected_goal = GripperApplyEffort.Goal()
        expected_goal.effort = effort
        # action_client_mock.send_goal_async.assert_called_with(expected_goal)

    def test_gripper_apply_force_effort_negative_value(self):
        """Test apply force(negative effort is set)"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        assert_raises(hsrb_interface.exceptions.GripperError,
                      gripper.apply_force, -1.0)

    def test_gripper_apply_force_fail(self):
        """Test apply force(failed to apply force)"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = True
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_CANCELING
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        # assert_raises(hsrb_interface.exceptions.GripperError,
        #               gripper.apply_force, 1.0)

    def test_gripper_apply_force_time_out(self):
        """Test apply force(timed out)"""
        robot = Robot()
        self.get_entry_mock.side_effect = [self.gripper_setting]
        action_client_mock = self.action_client_mock.return_value
        action_client_mock.wait_for_result.return_value = False
        gripper = hsrb_interface.end_effector.Gripper('foo',robot)
        # assert_raises(hsrb_interface.exceptions.GripperError,
        #              gripper.apply_force, 1.0)

    '''
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
    '''
        
