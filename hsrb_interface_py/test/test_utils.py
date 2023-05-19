#!/usr/bin/env python
# vim: fileencoding=utf-8
# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface_py.utils module."""

from hsrb_interface_py import _testing as testing
from hsrb_interface_py import utils

from nose.tools import eq_

from sensor_msgs.msg import JointState


class UtilsTestCase(testing.RosMockTestCase):

    def test_caching_subscriber(self):
        """Test CachingSubscriber class"""
        sub = utils.CachingSubscriber(
            "/whole_body/joint_states",
            JointState,
            self.node)
        self.subscriber_mock.assert_called_with(
            JointState,
            "/whole_body/joint_states",
            sub._callback,
            1)


def test_iterate():
    """Test iterate function."""
    data = list(utils.iterate(lambda: 2, 5))
    eq_(len(data), 5)
    for datum in data:
        eq_(2, datum)
