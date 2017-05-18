# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.mobile_base module"""
import actionlib

import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.geometry
import hsrb_interface.mobile_base

from mock import MagicMock
from mock import patch

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

from nose.tools import assert_almost_equal
from nose.tools import assert_raises
from nose.tools import eq_
from nose.tools import ok_

import tf

from tmc_manipulation_msgs.msg import MultiDOFJointTrajectory
from tmc_manipulation_msgs.msg import MultiDOFJointTrajectoryPoint


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base(mock_get_entry, mock_connecting,
                     mock_action_client_cls, mock_trajectory_controller):
    """Test simple use case of MobileBase class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    ok_(mobile_base)
    mock_get_entry.call_with_args("mobile_base", "omni_base")
    mock_action_client_cls.call_with_args("/move_base", MoveBaseAction)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_x_y_yaw(mock_get_entry, mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    """Test MobileBase.go"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED

    mock_get_entry.return_value = {
        'frame_id': 'map'
    }
    mobile_base.go(0, 1, 2, timeout=3.0)

    expected_goal = MoveBaseGoal()
    expected_goal.target_pose.header.frame_id = "map"
    expected_goal.target_pose.header.stamp = mock_time_cls(0)
    expected_goal.target_pose.pose.position.x = 0
    expected_goal.target_pose.pose.position.y = 1
    q = tf.transformations.quaternion_from_euler(0.0, 0.0, 2)
    expected_goal.target_pose.pose.orientation.x = q[0]
    expected_goal.target_pose.pose.orientation.y = q[1]
    expected_goal.target_pose.pose.orientation.z = q[2]
    expected_goal.target_pose.pose.orientation.w = q[3]

    mock_action_client.send_goal.call_with_args(expected_goal)
    mock_action_client.wait_for_result.call_with_args(mock_duraiton_cls(3.0))


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_pos_ori(mock_get_entry, mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    """Test MobileBase.move"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    mock_action_client.get_state.return_value = actionlib.GoalStatus.SUCCEEDED

    mock_get_entry.return_value = {
        'frame_id': 'map'
    }
    pose = ((0, 1, 2), (0.5, 0.5, 0.5, 0.5))
    mobile_base.move(pose, timeout=3.0, ref_frame_id="map")

    expected_goal = MoveBaseGoal()
    expected_goal.target_pose.header.frame_id = "map"
    expected_goal.target_pose.header.stamp = mock_time_cls(0)
    expected_goal.target_pose.pose.position.x = 0
    expected_goal.target_pose.pose.position.y = 1
    expected_goal.target_pose.pose.position.z = 2
    expected_goal.target_pose.pose.orientation.x = 0.5
    expected_goal.target_pose.pose.orientation.y = 0.5
    expected_goal.target_pose.pose.orientation.z = 0.5
    expected_goal.target_pose.pose.orientation.w = 0.5

    mock_action_client.send_goal.call_with_args(expected_goal)
    mock_action_client.wait_for_result.call_with_args(mock_duraiton_cls(3.0))


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_get_pose(mock_get_entry, mock_connecting,
                              mock_action_client_cls,
                              mock_trajectory_controller):
    """Test MobileBase.get_pose()"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    ok_(mobile_base)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_go_failure(mock_get_entry,
                                mock_connecting,
                                mock_action_client_cls,
                                mock_trajectory_controller):
    """Test MobileBase.go faile if timeout is invalid"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    assert_raises(ValueError, mobile_base.go, 0, 0, 0, -1)
    assert_raises(ValueError, mobile_base.go, 0, 0, 0, float("inf"))
    assert_raises(ValueError, mobile_base.go, 0, 0, 0, float("nan"))


@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.timeopt_filter")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_poses(mock_get_entry,
                                  mock_connecting,
                                  mock_action_client_cls,
                                  mock_trajectory_controller,
                                  mock_get_frame,
                                  mock_transform_trajectory,
                                  mock_timeopt_filter,
                                  mock_wait_controllers):
    """Test MobileBase.follow with poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"
    mock_transform_trajectory.return_value = "piyo"
    mock_timeopt_filter.return_value = "foo"

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow(poses)

    mock_get_frame.assert_called_with("map")
    mock_timeopt_filter.assert_called_with("piyo")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_follow_client.submit.assert_called_with("foo")
    mock_wait_controllers.assert_called_with([mock_follow_client])

    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "hoge")
    eq_(len(trajectory.points), 3)
    assert_almost_equal(trajectory.points[0].transforms[0].translation.x, 2.0)
    assert_almost_equal(trajectory.points[1].transforms[0].translation.x, 1.0)
    assert_almost_equal(trajectory.points[2].transforms[0].translation.x, 0.0)

    # Set ref_frame_id
    mock_get_frame.reset_mock()
    mobile_base.follow(poses, ref_frame_id="var")

    mock_get_frame.assert_not_called()
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "var")


@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_stamped_poses(mock_get_entry,
                                          mock_connecting,
                                          mock_action_client_cls,
                                          mock_trajectory_controller,
                                          mock_get_frame,
                                          mock_transform_trajectory,
                                          mock_wait_controllers):
    """Test MobileBase.follow with stamped poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base",
        "follow_trajectory_action": "/follow_trajectory"
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"

    trajectory = MultiDOFJointTrajectory()
    for index in range(3):
        trajectory.points.append(MultiDOFJointTrajectoryPoint())
    mock_transform_trajectory.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow(poses, [3.0, 6.0])

    mock_get_frame.assert_called_with("map")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_wait_controllers.assert_called_with([mock_follow_client])

    submitted_trajectory = mock_follow_client.submit.call_args[0][0]
    eq_(len(submitted_trajectory.points), 3)
    assert_almost_equal(
        submitted_trajectory.points[0].time_from_start.to_sec(), 0.0)
    assert_almost_equal(
        submitted_trajectory.points[1].time_from_start.to_sec(), 3.0)
    assert_almost_equal(
        submitted_trajectory.points[2].time_from_start.to_sec(), 6.0)

    # Length of time_from_starts and poses should be same
    assert_raises(ValueError, mobile_base.follow, poses, [3.0])
    assert_raises(ValueError, mobile_base.follow, poses, [0.0, 3.0, 6.0])
