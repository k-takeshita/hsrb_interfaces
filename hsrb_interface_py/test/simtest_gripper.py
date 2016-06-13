#!/usr/bin/env python
# vim: fileencoding=utf-8 :
"""Test gripper object.

"""

import hsrb_interface

import testing

class GripperTest(testing.HsrbInterfaceTest):
    def test_command_gripper(self):
        """Driving gripper"""
        self.whole_body.move_to_neutral()
        self.assert_joints_reach_goals(self.EXPECTED_NEUTRAL, 0.01)

        self.gripper.command(1.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.command(0.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

        self.gripper.command(1.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.grasp(-0.01)
        self.assert_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'simtest_gripper', GripperTest)
