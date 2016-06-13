#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""

"""
import unittest
from tf import transformations
from hsrb_interface import geometry
import testing

class OmniBaseTest(testing.HsrbInterfaceTest):

    def test_simple_move(self):
        """Getting started"""
        self.omni_base.go(0.1, 0.0, 0.0, self.BASE_MOVE_TIME_TOLERANCE,
                          relative=True)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(0.1, 0.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

        self.omni_base.go(-0.1, 0.0, 0.0, self.BASE_MOVE_TIME_TOLERANCE,
                          relative=True)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(0.0, 0.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

    def test_get_current_position(self):
        """Get current position"""
        self.whole_body.move_to_go()
        self.omni_base.go(0, 0, 0, self.BASE_MOVE_TIME_TOLERANCE)

        pose2d = self.omni_base.pose
        pose3d = self.omni_base.get_pose('map')
        self.assertAlmostEqual(pose2d[0], pose3d[0].x)
        self.assertAlmostEqual(pose2d[1], pose3d[0].y)
        roll, pitch, yaw = transformations.euler_from_quaternion(pose3d[1])
        self.assertAlmostEqual(0, roll)
        self.assertAlmostEqual(0, pitch)
        self.assertAlmostEqual(pose2d[2], yaw)

    def test_move_globally(self):
        """Move to global position"""
        self.whole_body.move_to_go()
        self.omni_base.go(6.0, 6.0, 0.0, 300.0, relative=False)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(6.0, 6.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

        self.omni_base.go(0.0, 0.0, 0.0, 300.0, relative=False)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(0.0, 0.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)
    #@unittest.skip('test')
    def test_move_relatively(self):
        """Move to relative position"""
        self.whole_body.move_to_go()

        self.omni_base.go(0.0, 0.0, 0.0, 300.0, relative=False)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(0.0, 0.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

        self.omni_base.go(0.0, 1.0, 0.0, 300.0, relative=True)
        self.expect_pose_equal(self.omni_base.get_pose('map'),
                               geometry.pose(0.0, 1.0, 0.0),
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

    @unittest.skip('test')
    def test_move_using_tf(self):
        """Relative move using tf"""
        self.whole_body.move_to_go()

        goal = geometry.pose(ej=-1.57)
        self.omni_base.move(goal, 100.0, ref_frame_id='my_frame')
        self.expect_pose_equal(self.omni_base.get_pose('my_frame'), goal,
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

        goal = geometry.pose(z=-0.5, ej=-1.57)
        self.omni_base.move(goal, 100.0, ref_frame_id='my_frame')
        self.expect_pose_equal(self.omni_base.get_pose('my_frame'), goal,
                               delta=self.BASE_MOVE_GOAL_TOLERANCE)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_omni_base', OmniBaseTest)
