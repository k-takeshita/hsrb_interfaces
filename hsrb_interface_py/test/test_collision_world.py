from nose.tools import ok_, eq_
from mock import patch, call, PropertyMock

from sensor_msgs.msg import (
    Image,
    Imu,
    LaserScan,
)
from geometry_msgs.msg import WrenchStamped
import hsrb_interface

@patch('hsrb_interface.settings.get_entry')
def test_creation(mock_get_entry):
    pass


