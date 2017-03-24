# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for end_effector module"""

import hsrb_interface
import hsrb_interface.end_effector

from mock import patch
from nose.tools import assert_raises
from nose.tools import ok_

from std_msgs.msg import Bool


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
def test_gripper(mock_get_entry, mock_connecting):
    """Test Gripper class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'topic': 'hoge',
        "joint_names": ["hand_motor_joint"],
        "prefix": "/hsrb/gripper_controller"
    }
    gripper = hsrb_interface.end_effector.Gripper('foo')
    ok_(gripper)


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
def test_suction(mock_sub_class, mock_get_entry, mock_connecting):
    """Test Suction class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "action": "/suction_control",
        "suction_topic": "/suction_on",
        "pressure_sensor_topic": "/pressure_sensor"
    }
    suction = hsrb_interface.end_effector.Suction('foo')
    ok_(suction)
    mock_get_entry.assert_called_with('end_effector', 'foo')


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
@patch('rospy.Publisher')
def test_suction_command(mock_pub_class, mock_sub_class,
                         mock_get_entry, mock_connecting):
    """Test Suction command(exceptions are never thrown)"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "action": "/suction_control",
        "suction_topic": "/suction_on",
        "pressure_sensor_topic": "/pressure_sensor"
    }
    suction = hsrb_interface.end_effector.Suction('foo')
    suction.command(True)
    suction.command(False)
    mock_pub_class.assert_called_with("/suction_on", Bool, queue_size=0)


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
def test_suction_command_negative_num(mock_sub_class,
                                      mock_get_entry,
                                      mock_connecting):
    """Test Suction command(negative number)"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        "action": "/suction_control",
        "suction_topic": "/suction_on",
        "pressure_sensor_topic": "/pressure_sensor"
    }
    suction = hsrb_interface.end_effector.Suction('foo')
    assert_raises(ValueError, suction.command, -1)
