#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""
import hsrb_interface
import rospy
import unittest

from nose.tools import eq_
from nose.tools import ok_


class DuplicateInitTest(unittest.TestCase):
    def test_duplicate_init(self):
        """Test duplicate init"""
        rospy.init_node('test_node_name')
        robot = hsrb_interface.Robot()
        eq_('test_node_name', rospy.client._init_node_args[0])

        
if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'robot-duplicate-init-test', DuplicateInitTest)

