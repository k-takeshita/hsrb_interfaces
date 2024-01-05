#!/usr/bin/env python
# vim: fileencoding=utf-8
# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface_py.utils module."""

import _testing as testing
from hsrb_interface import Robot
from hsrb_interface import utils
from nose.tools import eq_
import rclpy


from sensor_msgs.msg import JointState


class UtilsTestCase(testing.RosMockTestCase):

    def test_caching_subscriber(self):
        rclpy.init()
        robot = Robot()
        """Test CachingSubscriber class"""
        sub = utils.CachingSubscriber(
            "/joint_states",
            JointState)
        self.subscriber_mock.assert_called_with(
            JointState,
            "/joint_states",
            sub._callback,
            1)


def test_iterate():
    """Test iterate function."""
    data = list(utils.iterate(lambda: 2, 5))
    eq_(len(data), 5)
    for datum in data:
        eq_(2, datum)
