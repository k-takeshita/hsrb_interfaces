# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.mobile_base module"""
import warnings

from geometry_msgs.msg import PoseStamped
import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.geometry
import hsrb_interface.mobile_base
import action_msgs.msg as action_msgs
from hsrb_interface import Robot
from mock import MagicMock
from mock import patch
from nose.tools import assert_almost_equal
from nose.tools import assert_false
from nose.tools import assert_raises
from nose.tools import assert_true
from nose.tools import eq_
from nose.tools import ok_
import tf_transformations
import rclpy
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import FollowPath


from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


@patch('hsrb_interface.settings.get_entry')
@patch("hsrb_interface.Robot._connecting")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.trajectory.TrajectoryController")
def test_mobile_base(mock_trajectory_controller,
                     mock_connecting,
                     mock_action_client_cls,
                     mock_get_entry):
    rclpy.init() 
    robot = Robot()

    """Test simple use case of MobileBase class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    ok_(mobile_base)
    mock_action_client_cls.call_with_args(robot._conn, NavigateToPose,"/navigate_to_pose")
    
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_x_y_yaw(mock_get_entry, mock_get_frame,
                                  mock_connecting,
                                  mock_spin_until_future_complete_cls,
                                  mock_action_client_goal_handle_cls,
                                  mock_action_client_future_result_cls,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):

    robot = Robot()
    """Test MobileBase.go_abs and MobileBase.go_rel"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_get_frame.return_value = "hoge"
    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()

    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)

    mobile_base.go_abs(0.0, 1.0, 2.0, timeout=3.0)

    expected_goal = NavigateToPose.Goal()
    expected_goal.pose.header.frame_id = "hoge"
    expected_goal.pose.header.stamp = robot._conn.get_clock().now().to_msg()
    expected_goal.pose.pose.position.x = 0.0
    expected_goal.pose.pose.position.y = 1.0
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, 2.0)
    expected_goal.pose.pose.orientation.x = q[0]
    expected_goal.pose.pose.orientation.y = q[1]
    expected_goal.pose.pose.orientation.z = q[2]
    expected_goal.pose.pose.orientation.w = q[3]

    mock_get_frame.assert_called_with("map")
    send_goal_future = mock_action_client.send_goal_async.call_with_args(expected_goal)

    mobile_base.go_rel(0.0, 1.0, 2.0, timeout=3.0)
    mock_get_frame.assert_called_with("base")

    warnings.simplefilter("always")
    with warnings.catch_warnings(record=True) as w:
        mobile_base.go(0.0, 1.0, 2.0, timeout=3.0, relative=False)
        mock_get_frame.assert_called_with("map")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)

    with warnings.catch_warnings(record=True) as w:
        mobile_base.go(0.0, 1.0, 2.0, timeout=3.0, relative=True)
        mock_get_frame.assert_called_with("base")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)

    


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_pos_ori(mock_get_entry, mock_get_frame,
                                  mock_connecting,
                                  mock_spin_until_future_complete_cls,
                                  mock_action_client_goal_handle_cls,
                                  mock_action_client_future_result_cls,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    robot = Robot()
    """Test MobileBase.go_pose"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }

    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()

    mock_action_client = mock_action_client_cls.return_value
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)


    pose = ((0.0, 1.0, 2.0), (0.5, 0.5, 0.5, 0.5))
    mobile_base.go_pose(pose, timeout=3.0, ref_frame_id="map")

    expected_goal = NavigateToPose.Goal()
    expected_goal.pose.header.frame_id = "map"
    expected_goal.pose.header.stamp = robot._conn.get_clock().now().to_msg()
    expected_goal.pose.pose.position.x = 0.0
    expected_goal.pose.pose.position.y = 1.0
    expected_goal.pose.pose.position.z = 2.0
    expected_goal.pose.pose.orientation.x = 0.5
    expected_goal.pose.pose.orientation.y = 0.5
    expected_goal.pose.pose.orientation.z = 0.5
    expected_goal.pose.pose.orientation.w = 0.5

    mock_action_client.send_goal.call_with_args(expected_goal)
    mock_action_client.wait_for_result.call_with_args(mock_duraiton_cls(3.0))

    warnings.simplefilter("always")
    with warnings.catch_warnings(record=True) as w:
        mobile_base.move(pose, timeout=3.0, ref_frame_id="map")
        eq_(w[0].category, hsrb_interface.exceptions.DeprecationWarning)


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_get_pose(mock_get_entry, mock_connecting,
                              mock_action_client_cls,
                              mock_trajectory_controller):

    robot = Robot()

    """Test MobileBase.get_pose()"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }

    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    ok_(mobile_base)

@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_go_failure(mock_get_entry,
                                mock_connecting,
                                mock_action_client_cls,
                                mock_trajectory_controller):
    robot = Robot()
    """Test MobileBase.go faile if timeout is invalid"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)

    assert_raises(ValueError, mobile_base.go_abs, 0.0, 0.0, 0.0, -1.0)
    assert_raises(ValueError, mobile_base.go_abs, 0.0, 0.0, 0.0, float("inf"))
    assert_raises(ValueError, mobile_base.go_abs, 0.0, 0.0, 0.0, float("nan"))

    assert_raises(ValueError, mobile_base.go_rel, 0.0, 0.0, 0.0, -1.0)
    assert_raises(ValueError, mobile_base.go_rel, 0.0, 0.0, 0.0, float("inf"))
    assert_raises(ValueError, mobile_base.go_rel, 0.0, 0.0, 0.0, float("nan"))




@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.timeopt_filter")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_trajectory(mock_get_entry,
                                       mock_connecting,
                                       mock_action_client_cls,
                                       mock_trajectory_controller,
                                       mock_get_frame,
                                       mock_transform_trajectory,
                                       mock_timeopt_filter,
                                       mock_wait_controllers):
    robot = Robot()
    """Test MobileBase.follow with poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"
    mock_transform_trajectory.return_value = "piyo"
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_timeopt_filter.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow_trajectory(poses)

    mock_get_frame.assert_called_with("map")
    mock_timeopt_filter.assert_called_with("piyo")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_follow_client.submit.assert_called_with(trajectory)
    mock_wait_controllers.assert_called_with(mobile_base,[mock_follow_client])

    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "hoge")
    eq_(len(trajectory.points), 3)
    assert_almost_equal(trajectory.points[0].transforms[0].translation.x, 2.0)
    assert_almost_equal(trajectory.points[1].transforms[0].translation.x, 1.0)
    assert_almost_equal(trajectory.points[2].transforms[0].translation.x, 0.0)

    # Set ref_frame_id
    mock_get_frame.reset_mock()
    mobile_base.follow_trajectory(poses, ref_frame_id="var")

    mock_get_frame.assert_not_called()
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "var")


@patch("hsrb_interface.trajectory.wait_controllers")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_follow_trajectory_with_stamp(mock_get_entry,
                                                  mock_connecting,
                                                  mock_action_client_cls,
                                                  mock_trajectory_controller,
                                                  mock_get_frame,
                                                  mock_transform_trajectory,
                                                  mock_wait_controllers):

    robot = Robot()
    """Test MobileBase.follow with stamped poses"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"

    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_transform_trajectory.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    mobile_base.follow_trajectory(poses, [3.0, 6.0])

    mock_get_frame.assert_called_with("map")
    mock_follow_client = mock_trajectory_controller.return_value
    mock_wait_controllers.assert_called_with(mobile_base,[mock_follow_client])

    submitted_trajectory = mock_follow_client.submit.call_args[0][0]
    eq_(len(submitted_trajectory.points), 2)

    print(submitted_trajectory.points[0].time_from_start.sec)
    assert_almost_equal(
        float(submitted_trajectory.points[0].time_from_start.sec), 3.0)
    assert_almost_equal(
        float(submitted_trajectory.points[1].time_from_start.sec), 6.0)

    # Length of time_from_starts and poses should be same
    assert_raises(ValueError, mobile_base.follow_trajectory,
                  poses, [3.0])
    assert_raises(ValueError, mobile_base.follow_trajectory,
                  poses, [0.0, 3.0, 6.0])

@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_create_go_pose_goal(mock_get_entry,
                             mock_connecting,
                             mock_action_client_cls,
                             mock_trajectory_controller,
                             mock_get_frame):

    robot = Robot()
    """Test MobileBase.create_move_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)

    goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(x=1.0))
    eq_(goal.header.frame_id, "hoge")
    assert_almost_equal(goal.pose.position.x, 1.0)

    goal = mobile_base.create_go_pose_goal(hsrb_interface.geometry.pose(),
                                           "piyo")
    eq_(goal.header.frame_id, "piyo")
    assert_almost_equal(goal.pose.position.x, 0.0)


@patch("hsrb_interface.trajectory.timeopt_filter")
@patch("hsrb_interface.trajectory.transform_base_trajectory")
@patch("hsrb_interface.settings.get_frame")
@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.action.ActionClient")
@patch("hsrb_interface.Robot._connecting")
@patch('hsrb_interface.settings.get_entry')
def test_create_follow_goal(mock_get_entry,
                            mock_connecting,
                            mock_action_client_cls,
                            mock_trajectory_controller,
                            mock_get_frame,
                            mock_transform_trajectory,
                            mock_timeopt_filter):

    robot = Robot()
    """Test MobileBase.create_follow_trajectory_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_get_frame.return_value = "hoge"
    mock_transform_trajectory.return_value = "piyo"
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_timeopt_filter.return_value = trajectory

    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    mobile_base.get_pose = MagicMock()
    mobile_base.get_pose.return_value = hsrb_interface.geometry.pose(x=2.0)

    # Without time_from_starts
    poses = [hsrb_interface.geometry.pose(x=1.0),
             hsrb_interface.geometry.pose(x=0.0)]
    goal = mobile_base.create_follow_trajectory_goal(poses)

    mock_get_frame.assert_called_with("map")
    mock_timeopt_filter.assert_called_with("piyo")
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "hoge")
    eq_(len(trajectory.points), 3)
    assert_almost_equal(trajectory.points[0].transforms[0].translation.x, 2.0)
    assert_almost_equal(trajectory.points[1].transforms[0].translation.x, 1.0)
    assert_almost_equal(trajectory.points[2].transforms[0].translation.x, 0.0)

    # Set ref_frame_id
    mock_get_frame.reset_mock()
    goal = mobile_base.create_follow_trajectory_goal(poses, ref_frame_id="var")

    mock_get_frame.assert_not_called()
    trajectory = mock_transform_trajectory.call_args[0][0]
    eq_(trajectory.header.frame_id, "var")

    # With time_from_starts
    mock_get_frame.reset_mock()
    trajectory = JointTrajectory()
    for index in range(3):
        trajectory.points.append(JointTrajectoryPoint())
    mock_transform_trajectory.return_value = trajectory
    goal = mobile_base.create_follow_trajectory_goal(poses, [3.0, 6.0])

    mock_get_frame.assert_called_with("map")
    eq_(len(goal.points), 2)
    assert_almost_equal(float(goal.points[0].time_from_start.sec), 3.0)
    assert_almost_equal(float(goal.points[1].time_from_start.sec), 6.0)

    # Length of time_from_starts and poses should be same
    assert_raises(ValueError, mobile_base.create_follow_trajectory_goal,
                  poses, [3.0])
    assert_raises(ValueError, mobile_base.create_follow_trajectory_goal,
                  poses, [0.0, 3.0, 6.0])




@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_is_moving(mock_get_entry, mock_get_frame,
                                  mock_connecting,
                                  mock_spin_until_future_complete_cls,
                                  mock_action_client_goal_handle_cls,
                                  mock_action_client_future_result_cls,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls,
                                  mock_trajectory_controller):
    robot = Robot()

    """Test MobileBase.is_moving"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()

    # mobile_base.execute is not called
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    assert_false(mobile_base.is_moving())

    # Send pose
    mobile_base.execute(PoseStamped())
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_EXECUTING
    assert_true(mobile_base.is_moving())
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    assert_false(mobile_base.is_moving())

    # Send trajetory
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
    assert_true(mobile_base.is_moving())
    mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_SUCCEEDED
    assert_false(mobile_base.is_moving())



@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_is_succeeded(mock_get_entry, mock_get_frame,
                      mock_connecting,
                      mock_spin_until_future_complete_cls,
                      mock_action_client_goal_handle_cls,
                      mock_action_client_future_result_cls,
                      mock_action_client_cls,
                      mock_time_cls, mock_duraiton_cls,
                      mock_trajectory_controller):

    robot = Robot()
    """Test MobileBase.is_moving"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()


    # mobile_base.execute is not called
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    assert_false(mobile_base.is_succeeded())

    # Send pose
    mobile_base.execute(PoseStamped())
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_EXECUTING
    assert_false(mobile_base.is_succeeded())
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    assert_true(mobile_base.is_succeeded())

    # Send trajetory
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
    assert_false(mobile_base.is_succeeded())
    mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_SUCCEEDED
    assert_true(mobile_base.is_succeeded())


@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_cancel_goal(mock_get_entry, mock_get_frame,
                     mock_connecting,
                     mock_spin_until_future_complete_cls,
                     mock_action_client_goal_handle_cls,
                     mock_action_client_future_result_cls,
                     mock_action_client_cls,
                     mock_time_cls, mock_duraiton_cls,
                     mock_trajectory_controller):

    robot = Robot()
    """Test MobileBase.cancel_goal"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }
    mock_action_client = mock_action_client_cls.return_value
    mock_action_client.wait_for_server.return_value = True
    mock_follow_client = mock_trajectory_controller.return_value

    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()

    # Cancel without goal
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    mobile_base.cancel_goal()

    mock_action_client.cancel_goal.assert_not_called()
    mock_follow_client.cancel.assert_not_called()

    # Send pose and cancel
    mobile_base.execute(PoseStamped())
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_EXECUTING
    mobile_base.cancel_goal()

    mock_action_client_goal_handle_cls.cancel_goal_async.assert_called_once_with()
    mock_follow_client.cancel.assert_not_called()

    # Send trajectory and cancel
    mobile_base.execute(JointTrajectory())
    mock_follow_client.get_state.return_value = action_msgs.GoalStatus.STATUS_EXECUTING
    mobile_base.cancel_goal()

    mock_follow_client.cancel.assert_called_once_with()

@patch("hsrb_interface.trajectory.TrajectoryController")
@patch("rclpy.duration.Duration")
@patch("rclpy.time.Time")
@patch("rclpy.action.ActionClient")
@patch("rclpy.task.Future.result")
@patch("rclpy.action.client.ClientGoalHandle")
@patch("rclpy.spin_until_future_complete")
@patch("hsrb_interface.Robot._connecting")
@patch("hsrb_interface.settings.get_frame")
@patch('hsrb_interface.settings.get_entry')
def test_execute(mock_get_entry, mock_get_frame,
                 mock_connecting,
                 mock_spin_until_future_complete_cls,
                 mock_action_client_goal_handle_cls,
                 mock_action_client_future_result_cls,
                 mock_action_client_cls,
                 mock_time_cls, mock_duraiton_cls,
                 mock_trajectory_controller):

    """Test MobileBase.execute"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "navigation_action":               "/navigate_to_pose",
        "follow_path_action":               "/follow_path",
        "follow_trajectory_action":  "/omni_base_controller",
        "pose_topic":                "/global_pose",
        "goal_topic":                "/base_goal",
        "timeout":                   1.0
    }

    mock_action_client_future_result_cls.return_value = mock_action_client_goal_handle_cls
    mock_action_client_future_result_cls.return_value.accepted = True
    mock_action_client_future_result_cls.return_value.status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    mock_action_client_future_result_cls.return_value.get_result_async.return_value = rclpy.task.Future()

    mock_follow_client = mock_trajectory_controller.return_value

    robot = Robot()
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base',robot)
    mobile_base._action_client = mock_action_client_cls.return_value

    input_goal = PoseStamped()
    mobile_base.execute(input_goal)

    action_goal = mobile_base._action_client.send_goal_async.call_args[0][0]
    eq_(action_goal.pose, input_goal)

    mobile_base.execute(JointTrajectory())
    mock_follow_client.submit.assert_called_with(JointTrajectory())

    assert_raises(ValueError, mobile_base.execute, "hoge")

    
