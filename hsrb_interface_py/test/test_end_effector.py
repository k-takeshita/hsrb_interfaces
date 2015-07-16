from nose.tools import ok_, eq_, raises
from mock import patch, call

import hsrb_interface
import hsrb_interface.end_effector
from tmc_msgs.msg import BatteryState


@patch.object(hsrb_interface.Robot, 'connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('actionlib.SimpleActionClient')
def test_gripper(mock_action_client_class, mock_get_entry, mock_connecting):
    mock_connecting.return_value = True
    mock_sub_instance = mock_action_client_class.return_value
    mock_get_entry.return_value = {
        'topic': 'hoge'
    }
    gripper = hsrb_interface.end_effector.Gripper('foo')


@patch.object(hsrb_interface.Robot, 'connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
def test_gripper(mock_sub_class, mock_get_entry, mock_connecting):
    mock_connecting.return_value = True
    mock_sub_instance = mock_sub_class.return_value
    mock_get_entry.return_value = {
        "action":                         "/suction_control",
        "suction_topic":                  "/suction_on",
        "pressure_sensor_topic":          "/pressure_sensor"
    }
    battery = hsrb_interface.end_effector.Suction('foo')
    mock_get_entry.assert_called_with('end_effector', 'foo')

