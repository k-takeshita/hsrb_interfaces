#!/usr/bin/env python
import sys
import math
import unittest

import hsrb_interface
import hsrb_interface.robot
from hsrb_interface import geometry
import rospy


class TestUserManual(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = hsrb_interface.Robot()
        # Wait simluation clock starts
        now = rospy.Time.now()
        while now == rospy.Time(0):
            now = rospy.Time.now()
            rospy.sleep(1)
        try:
            rospy.wait_for_message('/static_distance_map', rospy.AnyMsg)
            rospy.wait_for_message('/static_obstacle_map', rospy.AnyMsg)
            rospy.wait_for_message('/dynamic_obstacle_map', rospy.AnyMsg)
        except:
            sys.exit("Failed to connect a simuluation robot. Aborted.")

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def assert_pose_equal(self, pose1, pose2, delta=None):
        print pose1, pose2
        pos1, rot1 = pose1
        pos2, rot2 = pose2
        self.assertAlmostEqual(pos1[0], pos2[0], delta=delta)
        self.assertAlmostEqual(pos1[1], pos2[1], delta=delta)
        self.assertAlmostEqual(pos1[2], pos2[2], delta=delta)
        self.assertAlmostEqual(rot1[0], rot2[0], delta=delta)
        self.assertAlmostEqual(rot1[1], rot2[1], delta=delta)
        self.assertAlmostEqual(rot1[2], rot2[2], delta=delta)
        self.assertAlmostEqual(rot1[3], rot2[3], delta=delta)

    def assert_dict_contains_subset(self, a, b, delta=None):
        for key in a:
            if key not in b:
                self.fail()
            else:
                self.assertAlmostEqual(a[key], b[key], delta=delta)

    def test_5_1_2_1(self):
        """Getting started"""
        omni_base = self.robot.get('omni_base')
        omni_base.go(0.1, 0.0, 0.0, 10.0, relative=True)

        self.assert_pose_equal(omni_base.get_pose('map'),
                               geometry.pose(0.1, 0.0, 0.0),
                               delta=0.01)

    def test_5_1_3_1(self):
        """Pose transition"""
        expected_neutral = {
            "arm_lift_joint": 0,
            "arm_flex_joint": 0,
            "arm_roll_joint": 0,
            "wrist_flex_joint": math.radians(-90),
            "wrist_roll_joint": 0,
            "head_pan_joint": 0,
            "head_tilt_joint": 0,
        }

        expected_to_go = {
            "arm_lift_joint": 0,
            "arm_flex_joint": 0,
            "arm_roll_joint": math.radians(-90),
            "wrist_flex_joint": math.radians(-90),
            "wrist_roll_joint": 0,
            "head_pan_joint": 0,
            "head_tilt_joint": 0,
        }

        whole_body = self.robot.get('whole_body')

        whole_body.move_to_neutral()
        joint_positions = whole_body.joint_positions
        self.assert_dict_contains_subset(expected_neutral,
                                         joint_positions,
                                         0.01)

        whole_body.move_to_go()
        joint_positions = whole_body.joint_positions
        self.assert_dict_contains_subset(expected_to_go,
                                         joint_positions,
                                         0.01)

        whole_body.move_to_neutral()
        joint_positions = whole_body.joint_positions
        self.assert_dict_contains_subset(expected_neutral,
                                         joint_positions,
                                         0.01)

    def test_5_1_3_2(self):
        """Driving joints"""
        whole_body = self.robot.get('whole_body')

        whole_body.move_to_neutral()

        expected = [
            'arm_flex_joint',
            'arm_lift_joint',
            'arm_roll_joint',
            'base_l_drive_wheel_joint',
            'base_r_drive_wheel_joint',
            'base_roll_joint',
            'hand_l_spring_proximal_joint',
            'hand_motor_joint',
            'hand_r_spring_proximal_joint',
            'head_pan_joint',
            'head_tilt_joint',
            'wrist_flex_joint',
            'wrist_roll_joint'
        ]
        self.assertListEqual(expected, whole_body.joint_names)

        whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        expected_pose = {'arm_lift_joint': 0.2}
        self.assert_dict_contains_subset(expected_pose,
                                         whole_body.joint_positions,
                                         0.01)

        whole_body.move_to_joint_positions({'head_pan_joint': 0.4,
                                            'head_tilt_joint': -0.2})
        expected_pose = {'head_pan_joint': 0.4,
                         'head_tilt_joint': -0.2}
        self.assert_dict_contains_subset(expected_pose,
                                         whole_body.joint_positions,
                                         0.01)

    def test_5_1_3_3(self):
        pass

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'test_user_manual', TestUserManual)
