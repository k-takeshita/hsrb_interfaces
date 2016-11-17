# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.object_detection"""

import hsrb_interface
from hsrb_interface import object_detection

from mock import patch
from nose.tools import ok_


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Subscriber')
def test_marker_detector(mock_sub_class, mock_get_entry, mock_connecting):
    """Test ObjectDetector class"""
    mock_connecting.return_value = True
    detector = object_detection.ObjectDetector("marker")
    ok_(detector)
    mock_get_entry.call_with_args("object_detection", "marker")
