# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.battery module"""

import hsrb_interface
import hsrb_interface.battery
from mock import patch
from nose.tools import eq_
from tmc_msgs.msg import BatteryState


@patch.object(hsrb_interface.Robot, '_connection')
@patch('hsrb_interface.settings.get_entry')
@patch('hsrb_interface.utils.CachingSubscriber')
def test_battery(mock_sub_class, mock_get_entry, mock_connection):
    """Test Battery class"""
    mock_connection.return_value = True
    mock_sub_instance = mock_sub_class.return_value
    mock_get_entry.return_value = {
        'topic': 'hoge'
    }
    battery = hsrb_interface.battery.Battery('foo')
    mock_get_entry.assert_called_with('power_supply', 'foo')
    mock_sub_class.assert_called_with('hoge', BatteryState)

    mock_sub_instance.data.power = 100.0
    mock_sub_instance.data.temperature = 25.0

    eq_(battery.charge, 100.0)
    eq_(battery.temperature, 25.0)
