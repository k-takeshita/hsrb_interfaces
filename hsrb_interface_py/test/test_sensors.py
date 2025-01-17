# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for sensor objects."""
from unittest.mock import patch

from geometry_msgs.msg import WrenchStamped
from hsrb_interface import Robot
import hsrb_interface.sensors
from nose.tools import eq_
import rclpy

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_camera(mock_get_entry, mock_connecting, mock_sub_class):
    rclpy.init()
    robot = Robot()  # noqa: F841

    """Test Camera class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': 'head_l_stereo_camera',
        'prefix': "/stereo_camera/left",
    }

    camera = hsrb_interface.sensors.Camera('example')
    mock_get_entry.assert_called_with('camera', 'example')

    mock_sub_class.assert_called_with(
        "/stereo_camera/left/image_raw", Image)
    mock_sub_instance = mock_sub_class.return_value

    msg = Image()
    msg.header.stamp.sec = 1
    msg.header.stamp.nanosec = 2
    msg.header.frame_id = 'map'
    msg.height = 16
    msg.width = 16
    msg.encoding = 'rgb8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = range(256)

    mock_sub_instance.data = msg
    image = camera.image
    eq_(image.to_ros(), msg)


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_force_torque(mock_get_entry, mock_connecting,
                      mock_sub_class):
    robot = Robot()  # noqa: F841
    """Test ForceTorque class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'raw_topic': "raw_wrench",
        'compensated_topic': "compensated_wrench",
        'reset_service': "reset_wrench",
    }

    force_torque = hsrb_interface.sensors.ForceTorque('example')
    mock_get_entry.assert_called_with('force_torque', 'example')

    mock_sub_class.assert_any_call("raw_wrench", WrenchStamped)
    mock_sub_class.assert_any_call("compensated_wrench", WrenchStamped)

    mock_sub_instance = mock_sub_class.return_value

    msg = WrenchStamped()
    msg.wrench.force.x = 0.0
    msg.wrench.force.y = 1.0
    msg.wrench.force.z = 2.0
    msg.wrench.torque.x = 3.0
    msg.wrench.torque.y = 4.0
    msg.wrench.torque.z = 5.0
    mock_sub_instance.data = msg

    wrench = force_torque.raw
    eq_(wrench, ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0)))
    wrench = force_torque.wrench
    eq_(wrench, ((0.0, 1.0, 2.0), (3.0, 4.0, 5.0)))

    eq_(None, force_torque.reset())


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_imu(mock_get_entry, mock_connecting, mock_sub_class):
    robot = Robot()  # noqa: F841

    """Test Imu class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'topic': "foo",
    }

    imu = hsrb_interface.sensors.IMU('example')
    mock_get_entry.assert_called_with('imu', 'example')

    mock_sub_class.assert_called_with("foo", Imu)
    mock_sub_instance = mock_sub_class.return_value

    msg = Imu()
    msg.orientation.x = 0.0
    msg.orientation.y = 1.0
    msg.orientation.z = 2.0
    msg.orientation.w = 3.0
    msg.angular_velocity.x = 4.0
    msg.angular_velocity.y = 5.0
    msg.angular_velocity.z = 6.0
    msg.linear_acceleration.x = 7.0
    msg.linear_acceleration.y = 8.0
    msg.linear_acceleration.z = 9.0

    mock_sub_instance.data = msg

    ori, angular_vel, linear_acc = imu.data
    eq_(ori, (0.0, 1.0, 2.0, 3.0))
    eq_(angular_vel, (4.0, 5.0, 6.0))
    eq_(linear_acc, (7.0, 8.0, 9.0))


@patch('hsrb_interface.utils.CachingSubscriber')
@patch('hsrb_interface.robot.Robot._connecting')
@patch('hsrb_interface.settings.get_entry')
def test_lidar(mock_get_entry, mock_connecting, mock_sub_class):
    robot = Robot()  # noqa: F841
    """Test Lidar class"""
    mock_connecting.return_value = True
    mock_get_entry.return_value = {
        'name': "example",
        'topic': "foo",
    }

    lidar = hsrb_interface.sensors.Lidar('example')
    mock_get_entry.assert_called_with('lidar', 'example')

    mock_sub_class.assert_called_with("foo", LaserScan)
    mock_sub_instance = mock_sub_class.return_value

    msg = LaserScan()
    mock_sub_instance.data = msg

    scan = lidar.scan

    eq_(scan.to_ros(), msg)
