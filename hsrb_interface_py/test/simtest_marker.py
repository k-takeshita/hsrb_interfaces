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

class MarkerTest(testing.HsrbInterfaceTest):
    def test_detect_marker(self):
        """Getting started"""
        self.whole_body.move_to_go()
        self.omni_base.go(0.0, 1.5, 0.0, relative=True)
        self.whole_body.move_to_go()
        self.expect_object(27, geometry.pose(1.35, 1.52, 0.7, ek=math.pi/2.0),
                           frame='map')

# [<Object: id=27 name=first_aid_box>]
# (Vector3(x=1.3491728965091132, y=1.5227257975060506, z=0.69915176702885851), Quaternion(x=-0.0048577857130201567, y=0.00070191881511043916, z=0.72363403218862299, w=0.69016642680325935))
# []


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_marker', MarkerTest)
