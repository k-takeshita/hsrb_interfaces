"""Unittest for hsrb_interface.trajectory module"""
from __future__ import absolute_import

import sys
import os

from mock import patch
from mock import call
from nose.tools import eq_

import actionlib
import copy
import rospy

from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from control_msgs.msg import FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from tmc_manipulation_msgs.msg import ArmManipulationErrorCodes
from tmc_manipulation_msgs.msg import ArmNavigationErrorCodes
from tmc_manipulation_msgs.msg import ArmNavigationErrorCodes
from tmc_manipulation_msgs.srv import FilterJointTrajectory
from tmc_manipulation_msgs.srv import FilterJointTrajectoryRequest
from tmc_manipulation_msgs.srv import FilterJointTrajectoryWithConstraints
from tmc_manipulation_msgs.srv import FilterJointTrajectoryWithConstraintsRequest
from tmc_manipulation_msgs.srv import SelectConfig
from tmc_manipulation_msgs.srv import SelectConfigRequest

from hsrb_interface import trajectory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import testing


class TrajectoryTestCase(testing.RosMockTestCase):
    def trajectory_fixture(self):
        """Create a example joint trajectory"""
        traj = JointTrajectory()
        traj.joint_names = ['a', 'b', 'c']
        point1 = JointTrajectoryPoint(positions=[0, 0, 0],
                                      velocities=[0, 0, 0],
                                      accelerations=[0, 0, 0],
                                      effort=[0, 0, 0],
                                      time_from_start=rospy.Duration(0, 0))
        point2 = JointTrajectoryPoint(positions=[1, 2, 3],
                                      velocities=[1, 2, 3],
                                      accelerations=[1, 2, 3],
                                      effort=[1, 2, 3],
                                      time_from_start=rospy.Duration(1, 0))
        point3 = JointTrajectoryPoint(positions=[2, 4, 6],
                                      velocities=[2, 4, 6],
                                      accelerations=[2, 4, 6],
                                      effort=[2, 4, 6],
                                      time_from_start=rospy.Duration(2, 0))
        traj.points.append(point1)
        traj.points.append(point2)
        traj.points.append(point3)
        return traj


class TrajectoryModuleTest(TrajectoryTestCase):
    def test_extract_ok(self):
        """Test hsrb_interface.trajectory.extract()"""
        traj = self.trajectory_fixture()
        joint_state = JointState()
        joint_state.name = ['d']
        joint_state.position = [42]
        joint_state.velocity = [43]
        joint_state.effort = [44]
        result = trajectory.extract(traj, ['a', 'd'], joint_state)

        eq_(['a', 'd'], result.joint_names)
        eq_([0, 42], result.points[0].positions)
        eq_([0, 0], result.points[0].velocities)
        eq_([0, 0], result.points[0].accelerations)
        eq_([0, 0], result.points[0].effort)
        eq_(rospy.Duration(0, 0), result.points[0].time_from_start)

        eq_([1, 42], result.points[1].positions)
        eq_([1, 0], result.points[1].velocities)
        eq_([1, 0], result.points[1].accelerations)
        eq_([1, 0], result.points[1].effort)
        eq_(rospy.Duration(1, 0), result.points[1].time_from_start)

        eq_([2, 42], result.points[2].positions)
        eq_([2, 0], result.points[2].velocities)
        eq_([2, 0], result.points[2].accelerations)
        eq_([2, 0], result.points[2].effort)
        eq_(rospy.Duration(2, 0), result.points[2].time_from_start)

    def test_merge_ok(self):
        """Test hsrb_interface.trajectory.merge()"""
        traj1 = JointTrajectory()
        traj1.joint_names = ['a', 'b']
        traj1.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=rospy.Duration(0, 0)),
            JointTrajectoryPoint(positions=[1, 2],
                                 velocities=[1, 2],
                                 accelerations=[1, 2],
                                 effort=[1, 2],
                                 time_from_start=rospy.Duration(1, 0)),
        ]
        traj2 = JointTrajectory()
        traj2.joint_names = ['c', 'd']
        traj2.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=rospy.Duration(3, 0)),
            JointTrajectoryPoint(positions=[3, 4],
                                 velocities=[3, 4],
                                 accelerations=[3, 4],
                                 effort=[3, 4],
                                 time_from_start=rospy.Duration(4, 0)),
        ]
        result = trajectory.merge(traj1, traj2)

        eq_(['a', 'b', 'c', 'd'], result.joint_names)
        eq_([0, 0, 0, 0], result.points[0].positions)
        eq_([0, 0, 0, 0], result.points[0].velocities)
        eq_([0, 0, 0, 0], result.points[0].accelerations)
        eq_([0, 0, 0, 0], result.points[0].effort)
        eq_(rospy.Duration(0, 0), result.points[0].time_from_start)
        eq_([1, 2, 3, 4], result.points[1].positions)
        eq_([1, 2, 3, 4], result.points[1].velocities)
        eq_([1, 2, 3, 4], result.points[1].accelerations)
        eq_([1, 2, 3, 4], result.points[1].effort)
        eq_(rospy.Duration(1, 0), result.points[1].time_from_start)

    def test_adjust_time_ok(self):
        """Test hsrb_interface.trajectory.adjust_time()

        FIXME: Add assertions that test velocities and accelerations
        """
        # Setup pre-conditions
        traj1 = JointTrajectory()
        traj1.joint_names = ['a', 'b']
        traj1.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=rospy.Duration(0, 0)),
            JointTrajectoryPoint(positions=[1, 2],
                                 velocities=[1, 2],
                                 accelerations=[1, 2],
                                 effort=[1, 2],
                                 time_from_start=rospy.Duration(1, 0)),
        ]
        traj2 = JointTrajectory()
        traj2.joint_names = ['c', 'd']
        traj2.points = [
            JointTrajectoryPoint(positions=[0, 0],
                                 velocities=[0, 0],
                                 accelerations=[0, 0],
                                 effort=[0, 0],
                                 time_from_start=rospy.Duration(3, 0)),
            JointTrajectoryPoint(positions=[3, 4],
                                 velocities=[3, 4],
                                 accelerations=[3, 4],
                                 effort=[3, 4],
                                 time_from_start=rospy.Duration(4, 0)),
        ]

        # Call the target method
        trajectory.adjust_time(traj1, traj2)

        # Check post-conditions
        eq_(rospy.Duration(0, 0), traj2.points[0].time_from_start)
        eq_(rospy.Duration(1, 0), traj2.points[1].time_from_start)

    def test_constraint_filter_ok(self):
        """Test hsrb_interface.trajectory.constraint_filter()"""
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            '/filter_trajectory',
            10.0,
        ]
        service_proxy_mock = self.service_proxy_mock.return_value
        result = service_proxy_mock.call.return_value
        result.error_code.val = ArmNavigationErrorCodes.SUCCESS
        traj = self.trajectory_fixture()

        # Call the target method
        trajectory.constraint_filter(traj)

        # Check post-conditions
        self.get_entry_mock.assert_has_calls([
            call('trajectory', 'constraint_filter_service'),
            call('trajectory', 'filter_timeout')
        ])
        self.service_proxy_mock.assert_called_with("/filter_trajectory",
                                                   FilterJointTrajectoryWithConstraints)
        req = FilterJointTrajectoryWithConstraintsRequest()
        req.trajectory = traj
        req.allowed_time = rospy.Duration(10.0)

        service_proxy_mock.call.assert_called_with(req)

    def test_timeopt_filter_ok(self):
        """Test hsrb_interface.trajectory.timeopt_filter()"""
        # Setup pre-conditions
        self.get_entry_mock.return_value = '/timeopt_filter'
        service_proxy_mock = self.service_proxy_mock.return_value
        result = service_proxy_mock.call.return_value
        result.error_code.val = ArmNavigationErrorCodes.SUCCESS
        traj = self.trajectory_fixture()

        # Call the target method
        trajectory.timeopt_filter(traj)

        # Check post-conditions
        self.get_entry_mock.assert_called_with('trajectory',
                                               'timeopt_filter_service')
        self.service_proxy_mock.assert_called_with("/timeopt_filter",
                                                   FilterJointTrajectory)
        req = FilterJointTrajectoryRequest()
        req.trajectory = traj
        service_proxy_mock.call.assert_called_with(req)


class TrajectoryControllerTest(TrajectoryTestCase):
    def test_creation_ok(self):
        # Create an instance of target class
        controller = trajectory.TrajectoryController("test")
        self.action_client_mock.assert_called_with(
            "test/follow_joint_trajectory",
            FollowJointTrajectoryAction)
        self.get_entry_mock.assert_called_with('trajectory', 'action_timeout')
        self.get_param_mock.assert_called_with("test/joints")

    def test_submit_ok(self):
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("test")
        traj = self.trajectory_fixture()
        controller.submit(traj)

        # Check post-conditions
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = traj
        action_client_mock.send_goal.assert_called_with(goal)

    def test_cancel_ok(self):
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("test")
        traj = self.trajectory_fixture()
        controller.cancel()

        # Check post-conditions
        action_client_mock.cancel.assert_called()

    def test_get_state_ok(self):
        action_client_mock = self.action_client_mock.return_value
        controller = trajectory.TrajectoryController("test")
        traj = self.trajectory_fixture()
        controller.get_state()

        # Check post-conditions
        action_client_mock.get_state.assert_called()

    def test_get_result_ok(self):
        action_client_mock = self.action_client_mock.return_value
        expected_result = FollowJointTrajectoryResult()
        expected_result.error_code = FollowJointTrajectoryResult.SUCCESSFUL
        action_client_mock.get_state.return_value = actionlib.GoalStatus.SUCCEEDED
        action_client_mock.get_result.return_value = copy.deepcopy(expected_result)
        controller = trajectory.TrajectoryController("test")
        traj = self.trajectory_fixture()
        result = controller.get_result()

        # Check post-conditions
        action_client_mock.get_state.assert_called()
        action_client_mock.get_result.assert_called()
        eq_(expected_result, result)


class ImpedanceControllerTest(TrajectoryTestCase):
    def test_creation_ok(self):
        # Create an instance of target class
        controller = trajectory.ImpedanceController("test")

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

