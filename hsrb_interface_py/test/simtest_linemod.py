#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""

"""
import math
import rospy
import unittest
from tf import transformations
from hsrb_interface import geometry
import testing

class LinemodTest(testing.HsrbInterfaceTest):
    def test_detect_bottle(self):
        """Getting started"""
        self.whole_body.move_to_go()
        self.omni_base.go(0.5, 1.5, 0.0, relative=True)
        self.whole_body.move_to_go()
        self.expect_object(44, geometry.pose(0.85, 0.02, 0.7, ek=math.pi/2.0),
                           pos_delta=0.2, ori_delta=float('inf'),
                           frame='base_footprint')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_linemod', LinemodTest)
