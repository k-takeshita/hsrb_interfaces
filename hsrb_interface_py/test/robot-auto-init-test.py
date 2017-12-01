#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""
import hsrb_interface
import rospy
import unittest

from nose.tools import eq_
from nose.tools import ok_


class AutoInitTest(unittest.TestCase):
    def test_auto_init(self):
        """Test auto init rospy"""
        robot = hsrb_interface.Robot()
        eq_('hsrb_interface_py', rospy.client._init_node_args[0])


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'robot-auto-init-test', AutoInitTest)
