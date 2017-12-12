#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""

import unittest

import hsrb_interface
import rospy


class DuplicateInitTest(unittest.TestCase):
    def test_duplicate_init(self):
        """Test duplicate init

        only check no throw.
        """
        rospy.init_node('test_node_name')
        hsrb_interface.Robot()


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'robot-duplicate-init-test',
                   DuplicateInitTest)
