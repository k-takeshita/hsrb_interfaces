"""Unittest for end_effector module"""
from mock import patch
from nose.tools import ok_

import hsrb_interface
import hsrb_interface.end_effector


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
    battery = hsrb_interface.end_effector.Suction('foo')
    ok_(battery)
    mock_get_entry.assert_called_with('end_effector', 'foo')
