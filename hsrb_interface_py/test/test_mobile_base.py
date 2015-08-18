from nose.tools import ok_, eq_, raises, assert_almost_equals, assert_raises
from mock import patch

import tf
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

import hsrb_interface
import hsrb_interface.exceptions
import hsrb_interface.mobile_base


@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot.connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base(mock_get_entry, mock_connecting, mock_action_client_cls):
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base"
    }
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')
    mock_get_entry.call_with_args("mobile_base", "omni_base")
    mock_action_client_cls.call_with_args("/move_base", MoveBaseAction)


@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot.connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_x_y_yaw(mock_get_entry, mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls):
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base"
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


@patch("rospy.Duration")
@patch("rospy.Time")
@patch("actionlib.SimpleActionClient")
@patch("hsrb_interface.Robot.connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_goto_pos_ori(mock_get_entry, mock_connecting,
                                  mock_action_client_cls,
                                  mock_time_cls, mock_duraiton_cls):
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base"
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


@patch("tf2_ros.Buffer")
@patch("hsrb_interface.Robot.connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_get_pose(mock_get_entry, mock_connecting, mock_tf_cls):
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base"
    }
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')


@patch("tf2_ros.Buffer")
@patch("hsrb_interface.Robot.connecting")
@patch('hsrb_interface.settings.get_entry')
def test_mobile_base_go_fail_with_invalid_timeout(mock_get_entry,
                                                   mock_connecting,
                                                   mock_tf_cls):
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "pose_topic": "/pose",
        "move_base_action": "/move_base"
    }
    mobile_base = hsrb_interface.mobile_base.MobileBase('omni_base')

    assert_raises(ValueError, mobile_base.go, 0, 0, 0, -1)
    assert_raises(ValueError, mobile_base.go, 0, 0, 0, float("inf"))
    assert_raises(ValueError, mobile_base.go, 0, 0, 0, float("nan"))

