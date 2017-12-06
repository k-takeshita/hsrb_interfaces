#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""

import unittest

import hsrb_interface
from nose.tools import ok_
import rospy


class AutoInitTest(unittest.TestCase):
    def test_auto_init(self):
        """Test auto init rospy"""
        """Test auto init rospy"""
        hsrb_interface.Robot()
        ok_(rospy.core.is_initialized())


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'robot-auto-init-test', AutoInitTest)
