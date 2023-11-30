# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.trajectory module"""
from __future__ import absolute_import

import copy
import os
import sys
import json

from hsrb_interface import Robot
from control_msgs.action import FollowJointTrajectory
from hsrb_interface import _testing as testing
from hsrb_interface import trajectory
from action_msgs.msg import GoalStatus
from mock import call
from nose.tools import eq_
import rclpy
from sensor_msgs.msg import JointState
from tmc_manipulation_msgs.msg import ArmNavigationErrorCodes
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from tmc_manipulation_msgs.srv import FilterJointTrajectoryWithConstraints
from tmc_manipulation_msgs.srv import SelectConfig
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class TrajectoryTestCase(testing.RosMockTestCase):

    def trajectory_fixture(self):
        """Create a example joint trajectory"""
        traj = JointTrajectory()
        traj.joint_names = ['a', 'b', 'c']
        point1 = JointTrajectoryPoint(positions=[0.0, 0.0, 0.0],
                                      velocities=[0.0, 0.0, 0.0],
                                      accelerations=[0.0, 0.0, 0.0],
                                      time_from_start=rclpy.duration.Duration(seconds=0.0).to_msg())
        point2 = JointTrajectoryPoint(positions=[1.0, 2.0, 3.0],
                                      velocities=[1.0, 2.0, 3.0],
                                      accelerations=[1.0, 2.0, 3.0],
                                      time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg())
        point3 = JointTrajectoryPoint(positions=[2.0, 4.0, 6.0],
                                      velocities=[2.0, 4.0, 6.0],
                                      accelerations=[2.0, 4.0, 6.0],
                                      time_from_start=rclpy.duration.Duration(seconds=2.0).to_msg())
        traj.points.append(point1)
        traj.points.append(point2)
        traj.points.append(point3)
        return traj

    def state_fixture(self):
        """Create a example joint state"""
        state = JointState()
        state.name = ['a', 'b', 'c']
        state.position = [0.0, 0.0, 0.0]
        state.velocity = [1.0, 2.0, 3.0]
        return state


class TrajectoryModuleTest(TrajectoryTestCase):

    def test_extract_ok(self):
        """Test hsrb_interface.trajectory.extract()"""
        traj = self.trajectory_fixture()
        joint_state = JointState()
        joint_state.name = ['d']
        joint_state.position = [42.0]
        joint_state.velocity = [43.0]
        result = trajectory.extract(traj, ['a', 'd'], joint_state)
        print(list(result.points[0].positions))
        eq_(['a', 'd'], result.joint_names)
        eq_([0.0, 42.0], list(result.points[0].positions))
        eq_([0.0, 0.0], list(result.points[0].velocities))
        eq_([0.0, 0.0], list(result.points[0].accelerations))
        eq_(rclpy.duration.Duration(seconds=0.0).to_msg(), result.points[0].time_from_start)

        eq_([1.0, 42.0], list(result.points[1].positions))
        eq_([1.0, 0.0], list(result.points[1].velocities))
        eq_([1.0, 0.0], list(result.points[1].accelerations))
        eq_(rclpy.duration.Duration(seconds=1.0).to_msg(), result.points[1].time_from_start)

        eq_([2.0, 42.0], list(result.points[2].positions))
        eq_([2.0, 0.0], list(result.points[2].velocities))
        eq_([2.0, 0.0], list(result.points[2].accelerations))
        eq_(rclpy.duration.Duration(seconds=2.0).to_msg(), result.points[2].time_from_start)

    def test_merge_ok(self):
        """Test hsrb_interface.trajectory.merge()"""
        traj1 = JointTrajectory()
        traj1.joint_names = ['a', 'b']
        traj1.points = [
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 accelerations=[0.0, 0.0],
                                 time_from_start=rclpy.duration.Duration(seconds=0.0).to_msg()),
            JointTrajectoryPoint(positions=[1.0, 2.0],
                                 velocities=[1.0, 2.0],
                                 accelerations=[1.0, 2.0],
                                 effort=[1.0, 2.0],
                                 time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg()),
        ]
        traj2 = JointTrajectory()
        traj2.joint_names = ['c', 'd']
        traj2.points = [
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 accelerations=[0.0, 0.0],
                                 time_from_start=rclpy.duration.Duration(seconds=3.0).to_msg()),
            JointTrajectoryPoint(positions=[3.0, 4.0],
                                 velocities=[3.0, 4.0],
                                 accelerations=[3.0, 4.0],
                                 time_from_start=rclpy.duration.Duration(seconds=4.0).to_msg()),
        ]
        result = trajectory.merge(traj1, traj2)

        eq_(['a', 'b', 'c', 'd'], result.joint_names)
        eq_([0.0, 0.0, 0.0, 0.0], list(result.points[0].positions))
        eq_([0.0, 0.0, 0.0, 0.0], list(result.points[0].velocities))
        eq_([0.0, 0.0, 0.0, 0.0], list(result.points[0].accelerations))
        eq_(rclpy.duration.Duration(seconds=0.0).to_msg(), result.points[0].time_from_start)
        eq_([1.0, 2.0, 3.0, 4.0], list(result.points[1].positions))
        eq_([1.0, 2.0, 3.0, 4.0], list(result.points[1].velocities))
        eq_([1.0, 2.0, 3.0, 4.0], list(result.points[1].accelerations))
        eq_(rclpy.duration.Duration(seconds=1.0).to_msg(), result.points[1].time_from_start)

    def test_adjust_time_ok(self):
        """Test hsrb_interface.trajectory.adjust_time()

        FIXME: Add assertions that test velocities and accelerations
        """
        # Setup pre-conditions
        traj1 = JointTrajectory()
        traj1.joint_names = ['a', 'b']
        traj1.points = [
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 accelerations=[0.0, 0.0],
                                 time_from_start=rclpy.duration.Duration(seconds=0.0).to_msg()),
            JointTrajectoryPoint(positions=[1.0, 2.0],
                                 velocities=[1.0, 2.0],
                                 accelerations=[1.0, 2.0],
                                 time_from_start=rclpy.duration.Duration(seconds=1.0).to_msg()),
        ]
        traj2 = JointTrajectory()
        traj2.joint_names = ['c', 'd']
        traj2.points = [
            JointTrajectoryPoint(positions=[0.0, 0.0],
                                 velocities=[0.0, 0.0],
                                 accelerations=[0.0, 0.0],
                                 time_from_start=rclpy.duration.Duration(seconds=3.0).to_msg()),
            JointTrajectoryPoint(positions=[3.0, 4.0],
                                 velocities=[3.0, 4.0],
                                 accelerations=[3.0, 4.0],
                                 time_from_start=rclpy.duration.Duration(seconds=4.0).to_msg()),
        ]

        # Call the target method
        trajectory.adjust_time(traj1, traj2)

        # Check post-conditions
        eq_(rclpy.duration.Duration(seconds=0.0).to_msg(), traj2.points[0].time_from_start)
        eq_(rclpy.duration.Duration(seconds=1.0).to_msg(), traj2.points[1].time_from_start)
    def test_constraint_filter_ok(self):
         
        rclpy.init()
        robot = Robot()
        """Test hsrb_interface.trajectory.constraint_filter()"""
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            '/filter_trajectory',
            10.0,
        ]
        service_client_mock = self.service_client_mock.return_value
        future = service_client_mock.call_async.return_value
        result = future.result.return_value
        result.error_code.val = ArmNavigationErrorCodes.SUCCESS
        traj = self.trajectory_fixture()
        # Call the target method
        trajectory.constraint_filter(traj,robot._conn)

        # Check post-conditions
        self.get_entry_mock.assert_has_calls([
            call('trajectory', 'constraint_filter_service'),
            call('trajectory', 'filter_timeout')
        ])
        self.service_client_mock.assert_called_with(
            FilterJointTrajectoryWithConstraints,
            "/filter_trajectory")
        req = FilterJointTrajectoryWithConstraints.Request()
        req.trajectory = traj
        req.allowed_time = rclpy.duration.Duration(seconds=10.0).to_msg()
        service_client_mock.call_async.assert_called_with(req)

    def test_timeopt_filter_ok(self):
        """Test hsrb_interface.trajectory.timeopt_filter()"""
        robot = Robot()
        # Setup pre-conditions
        self.get_entry_mock.return_value = '/timeopt_filter'
        service_client_mock = self.service_client_mock.return_value
        future = service_client_mock.call_async.return_value
        result = future.result.return_value
        result.error_code.val = ArmNavigationErrorCodes.SUCCESS
        traj = self.trajectory_fixture()

        # Call the target method
        trajectory.timeopt_filter(traj,robot._conn)

        # Check post-conditions
        self.get_entry_mock.assert_called_with('trajectory',
                                               'timeopt_filter_service')
        self.service_client_mock.assert_called_with(FilterJointTrajectory,
                                                    "/timeopt_filter")
        req = FilterJointTrajectory.Request()
        req.trajectory = traj
        service_client_mock.call_async.assert_called_with(req)

    def test_hsr_timeopt_filter_ok(self):
        """Test hsrb_interface.trajectory.hsr_timeopt_filter()"""
        robot = Robot()
        # Setup pre-conditions
        self.get_entry_mock.side_effect = ['/timeopt_filter', 'a']

        service_client_mock = self.service_client_mock.return_value
        future = service_client_mock.call_async.return_value
        result = future.result.return_value
        result.error_code.val = ArmNavigationErrorCodes.SUCCESS
        traj = self.trajectory_fixture()
        state = self.state_fixture()

        # Call the target method
        trajectory.hsr_timeopt_filter(traj, state,robot._conn)

        # Check post-conditions
        self.get_entry_mock.assert_has_calls([
            call('trajectory', 'whole_timeopt_filter_service'),
            call('trajectory', 'caster_joint')
        ])
        self.service_client_mock.assert_called_with(FilterJointTrajectory,
                                                   "/timeopt_filter")
        req = FilterJointTrajectory.Request()
        req.trajectory = traj
        whole_name = traj.joint_names + ['a']
        req.start_state.joint_state.name = whole_name
        whole_pos = [state.position[state.name.index(joint)]
                     for joint in whole_name]
        req.start_state.joint_state.position = whole_pos

        service_client_mock.call_async.assert_called_with(req)

class TrajectoryControllerTest(TrajectoryTestCase):

    def test_creation_ok(self):
        robot = Robot()
        # Create an instance of target class
        self.get_entry_mock.return_value = 30.0

        trajectory.TrajectoryController("arm_trajectory_controller",robot)
        self.get_entry_mock.assert_called_with('trajectory', 'action_timeout')
        self.get_param_mock.assert_called_with(robot._conn, 
                                               "arm_trajectory_controller/get_parameters",
                                               ["joints"])


    def test_submit_ok(self):
        robot = Robot()

        self.get_entry_mock.return_value = 30.0
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("arm_trajectory_controller",robot)
        traj = self.trajectory_fixture()
        controller.submit(traj)

        # Check post-conditions
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

    def test_cancel_ok(self):
        robot = Robot()

        self.get_entry_mock.return_value = 30.0
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("arm_trajectory_controller",robot)
        traj = self.trajectory_fixture()
        controller.submit(traj)
        controller.cancel()

    def test_get_state_ok(self):
        robot = Robot()
        self.get_entry_mock.return_value = 30.0
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("arm_trajectory_controller",robot)
        traj = self.trajectory_fixture()
        controller.submit(traj)
        controller.get_state()

    def test_get_result_ok(self):
        robot = Robot()
        self.get_entry_mock.return_value = 30.0
        action_client_mock = self.action_client_mock.return_value
        
        expected_result = FollowJointTrajectory.Result()
        expected_result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        action_client_mock.get_state.return_value = \
            GoalStatus.STATUS_SUCCEEDED
        action_client_mock.get_result.return_value = copy.deepcopy(
            expected_result)
        controller = trajectory.TrajectoryController("arm_trajectory_controller",robot)
        traj = self.trajectory_fixture()
        controller.submit(traj)
        # result = controller.get_result()

"""
class ImpedanceControllerTest(TrajectoryTestCase):

    def test_creation_ok(self):
        # Create an instance of target class
        trajectory.ImpedanceController("test")

        # Check post-conditions
        self.action_client_mock.assert_called_with(
            "test/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        self.get_entry_mock.assert_called_with('trajectory', 'action_timeout')
        self.get_param_mock.assert_has_calls([
            call("test/joint_names"),
            call("test/config_names", [])
        ])

    def test_submit_ok(self):
        action_client_mock = self.action_client_mock.return_value
        service_proxy_mock = self.service_proxy_mock.return_value
        self.get_param_mock.side_effect = [
            None,
            ['full_power'],
        ]
        controller = trajectory.ImpedanceController("test")

        # Call
        traj = self.trajectory_fixture()
        controller.config = 'full_power'
        controller.submit(traj)

        # Check post-conditions
        self.service_proxy_mock.assert_called_with("test/select_config",
                                                   SelectConfig)
        req = SelectConfigRequest()
        req.name = 'full_power'
        service_proxy_mock.call.assert_called_with(req)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        action_client_mock.send_goal.assert_called_with(goal)        
"""

