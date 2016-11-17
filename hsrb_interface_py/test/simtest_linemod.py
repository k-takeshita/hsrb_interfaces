#!/usr/bin/env python
# vim: fileencoding=utf-8 :
# Copyright (C) 2016 Toyota Motor Corporation
"""Testing LINEMOD interface in Gazebo simulator.

Target objects are spawen within simtest_linemod.test launch file.
"""
import math

from hsrb_interface import _testing as testing
from hsrb_interface import geometry


class LinemodTest(testing.HsrbInterfaceTest):
    """Test cases for LINEMOD detector."""

    def test_detect_bottle(self):
        """The robot should detect a bottle object by LINEMOD"""
        self.whole_body.move_to_go()
        self.omni_base.go(0.5, 1.5, 0.0, relative=True)
        self.whole_body.move_to_go()
        expected_pose = geometry.pose(0.85, 0.02, 0.7, ek=math.pi / 2.0)
        self.expect_detected_object(44, expected_pose,
                                    pos_delta=0.2, ori_delta=float('inf'),
                                    frame='base_footprint')

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_linemod', LinemodTest)
