from nose.tools import ok_, eq_, raises
from mock import patch, call

import hsrb_interface
import hsrb_interface.robot
import hsrb_interface.exceptions

@raises(hsrb_interface.exceptions.RobotConnectionError)
def test_resource():
    r = hsrb_interface.robot.Resource()
#

@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_lifecycle(init_mock, shutdown_mock):
    r = hsrb_interface.Robot()
    init_mock.assert_called_with('hsrb_interface_py', anonymous=True)
    eq_(r.ok(), True)
    r.close()
    shutdown_mock.assert_called_with('shutdown')
    eq_(r.ok(), False)


@patch('rospy.signal_shutdown')
@patch('rospy.init_node')
def test_robot_with_statement(init_mock, shutdown_mock):
    with hsrb_interface.Robot() as r:
        eq_(r.ok(), True)
        init_mock.assert_called_with('hsrb_interface_py', anonymous=True)
    shutdown_mock.assert_caleed_with('shutdown')






