# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest hsrb_interface.robot module."""
import hsrb_interface
import hsrb_interface.robot
from mock import patch
from nose.tools import eq_
from nose.tools import ok_
from nose.tools import raises


@raises(hsrb_interface.exceptions.RobotConnectionError)
def test_resource():
    """Test resource acquisition."""
    robot = hsrb_interface.robot.Item()
    ok_(robot)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_lifecycle(init_mock, shutdown_mock, mock_get_master,
                         mock_buffer, mock_listener):
    """Test basic lifecycle"""
    ok_(mock_get_master)
    ok_(mock_buffer)
    ok_(mock_listener)
    robot = hsrb_interface.Robot()
    init_mock.assert_called_with('hsrb_interface_py', disable_signals=False,
                                 anonymous=True)
    eq_(robot.ok(), True)
    robot.close()
    shutdown_mock.assert_called_with('shutdown')
    eq_(robot.ok(), False)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_with_statement(init_mock, shutdown_mock, mock_get_master,
                              mock_buffer, mock_listener):
    """Test use in with statement"""
    ok_(mock_get_master)
    ok_(mock_buffer)
    ok_(mock_listener)
    with hsrb_interface.Robot() as robot:
        eq_(robot.ok(), True)
        init_mock.assert_called_with('hsrb_interface_py',
                                     disable_signals=False, anonymous=True)
    shutdown_mock.assert_called_with('shutdown')


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('tf2_ros.BufferClient')
@patch('rospy.get_master')
@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_with_tf_client(init_mock, shutdown_mock, mock_get_master,
                              mock_buffer_client, mock_buffer, mock_listener):
    """Test use in tf client"""
    with hsrb_interface.Robot(use_tf_client=True) as robot:
        eq_(robot.ok(), True)
        mock_buffer_client.assert_called_with('/tf2_buffer_server')
        mock_buffer.assert_not_called()
        mock_listener.assert_not_called()

    shutdown_mock.assert_called_with('shutdown')
