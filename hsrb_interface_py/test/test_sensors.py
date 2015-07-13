from nose.tools import ok_, eq_
from mock import patch, call, PropertyMock

from sensor_msgs.msg import Image
import hsrb_interface

#@patch('cv_bridge.CvBridge')
#@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot.initialized')
@patch('hsrb_interface.settings.get_setting')
def test_camera(get_setting_mock, initialized_mock):#, sub_mock_class, cv_bridge_class):
    initialized_mock.return_value = True
    get_setting_mock.return_value = {
        'name':   'head_l_stereo_camera',
        'prefix': "/stereo_camera/left",
    }

    camera = hsrb_interface.Camera('example')
    get_setting_mock.assert_called_with('camera', 'example')

    sub_mock.assert_called_with("/stereo_camera/left/image_raw", Image)
    sub_instance = sub_mock.return_value

    msg = Image()
    msg.header.stamp.secs = 1
    msg.header.stamp.nsecs = 2
    msg.header.seq = 3
    msg.header.frame_id = 'map'
    msg.height = 16
    msg.width = 16
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = range(256)

    sub_instance.data = PropertyMock(return_value=msg)

    cv_image = camera.get_image()
    print cv_image






