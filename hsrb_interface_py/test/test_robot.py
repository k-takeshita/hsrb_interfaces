# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest hsrb_interface_py.robot module."""
from unittest.mock import patch

import hsrb_interface_py
import hsrb_interface_py.robot

from nose.tools import eq_
from nose.tools import ok_
from nose.tools import raises


@raises(hsrb_interface_py.exceptions.RobotConnectionError)
def test_resource():
    """Test resource acquisition."""
    robot = hsrb_interface_py.robot.Item()
    ok_(robot)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_lifecycle_close(mock_destroy, mock_init,
                               mock_buffer, mock_listener):
    """Test basic lifecycle"""
    ok_(mock_buffer)
    ok_(mock_listener)
    robot = hsrb_interface_py.Robot()
    mock_init.assert_called_with('hsrb_interface_py')
    eq_(robot.ok(), True)
    robot.close()
    mock_destroy.assert_called()
    eq_(robot.ok(), False)


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_lifecycle(mock_destroy, mock_init,
                         mock_buffer, mock_listener):
    """Test use in with statement"""
    ok_(mock_buffer)
    ok_(mock_listener)
    with hsrb_interface_py.Robot() as robot:
        eq_(robot.ok(), True)
        mock_init.assert_called_with('hsrb_interface_py')
    mock_destroy.assert_called()


@patch('tf2_ros.TransformListener')
@patch('tf2_ros.Buffer')
@patch('tf2_ros.BufferClient')
@patch('rclpy.node.Node.__init__')
@patch('rclpy.node.Node.destroy_node')
def test_robot_with_tf_client(mock_destroy, mock_init,
                              mock_buffer_client, mock_buffer, mock_listener):
    """Test use in tf client"""
    with hsrb_interface_py.Robot(use_tf_client=True) as robot:
        eq_(robot.ok(), True)
        mock_buffer_client.assert_called_with('/tf2_buffer_server')
        mock_buffer.assert_not_called()
        mock_listener.assert_not_called()

    mock_destroy.assert_called()
