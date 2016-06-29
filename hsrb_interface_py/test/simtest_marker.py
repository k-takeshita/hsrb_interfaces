#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""Testing streo marker detector interface in Gazebo simulator.

Target objects are spawen within simtest_linemod.test launch file.
"""
import math

from hsrb_interface import geometry
import testing


class MarkerTest(testing.HsrbInterfaceTest):
    """Test caes for marker detector."""

    def test_detect_marker(self):
        """The robot should detect a box object by stereo marker"""
        self.whole_body.move_to_go()
        self.omni_base.go(0.5, 1.5, 0.0, relative=True)
        self.whole_body.move_to_go()
        expected_pose = geometry.pose(0.85, 0.02, 0.7, ek=math.pi / 2.0)
        self.expect_object(27, expected_pose,
                           pos_delta=0.05, ori_delta=math.radians(5),
                           frame='base_footprint')


if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_marker', MarkerTest)
