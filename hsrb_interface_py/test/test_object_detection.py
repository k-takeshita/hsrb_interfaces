from nose.tools import ok_, eq_, raises
from mock import patch, call

import hsrb_interface
from hsrb_interface import object_detection


@patch.object(hsrb_interface.Robot, '_connecting')
@patch('hsrb_interface.settings.get_entry')
@patch('rospy.Time')
@patch('rospy.Duration')
@patch('rospy.Subscriber')
def test_marker_detector(mock_sub_class, mock_duration_class, mock_time_class,
                         mock_get_entry, mock_connecting):
    mock_connecting.return_value = True
    od = object_detection.ObjectDetector("marker")
    mock_get_entry.call_with_args("object_detection", "marker")
