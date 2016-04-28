#!/usr/bin/env python
"""Test tutorial codes in HSR Manual.

All test methods named after manual sections.
"""
import sys
import time
import copy
import math
import unittest

import hsrb_interface
import hsrb_interface.robot
from hsrb_interface import geometry
import rospy


def dict_almost_equal(a, b, delta):
    keys = a.keys()
    if sorted(keys) == sorted(b.keys()):
        return all(abs(a[key] - b[key]) < delta for key in keys)
    else:
        raise ValueError("a and b don't share keys")

EXPECTED_NEUTRAL = {
    "arm_lift_joint": 0,
    "arm_flex_joint": 0,
    "arm_roll_joint": 0,
    "wrist_flex_joint": math.radians(-90),
    "wrist_roll_joint": 0,
    "head_pan_joint": 0,
    "head_tilt_joint": 0,
}

EXPECTED_TO_GO = {
    "arm_lift_joint": 0,
    "arm_flex_joint": 0,
    "arm_roll_joint": math.radians(-90),
    "wrist_flex_joint": math.radians(-90),
    "wrist_roll_joint": 0,
    "head_pan_joint": 0,
    "head_tilt_joint": 0,
}

JOINT_NAMES = [
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

class TestUserManual(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = hsrb_interface.Robot()
        # Wait until simluation clock starts
        now = rospy.Time.now()
        while now == rospy.Time(0):
            now = rospy.Time.now()
            rospy.sleep(1)
        # Wait until navigation nodes start
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
        """
        """
        for key in a:
            if key not in b:
                self.fail()
            else:
                self.assertAlmostEqual(a[key], b[key], delta=delta)

    def wait_joints_stable(self, delta):
        """
        """
        whole_body = self.robot.get('whole_body')
        last_positions = whole_body.joint_positions
        while True:
            positions = whole_body.joint_positions
            if dict_almost_equal(positions, last_positions, delta):
                break
            else:
                last_positions = copy.deepcopy(positions)
                time.sleep(0.1)

    def assert_joints_reach_goals(self, expected, delta=None,
                                  timeout=30.0, tick=0.5):
        """Success if all `expected` joints reach their goals within `timeout`.

        Args:
            expected (Dict[str, float]): Target joints and their goals.
            delta (float): Threshold to a joint reach its goal.
            timeout (float): Timeout duration in seconds.
            tick (float): Sleep duration between each check.
        """
        whole_body = self.robot.get('whole_body')
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)
        if delta is None:
            delta = 0
        while True:
            num_reached = 0
            for joint, goal in expected.items():
                position = whole_body.joint_positions[joint]
                if abs(position - goal) <= delta:
                    num_reached += 1
            now = rospy.Time.now()
            if (now - start) > timeout:
                diffs = []
                positions = whole_body.joint_positions
                for key in expected:
                    template = '{0}: expected={1} actual={2}'
                    diff = template.format(key, expected[key], positions[key])
                    diffs.append(diff)
                self.fail('Timed out: {0}'.format(', '.join(diffs)))
            if num_reached == len(expected):
                break
            else:
                rospy.sleep(tick)

    def assert_end_effector_reach_goal(self, goal, frame='map', delta=None,
                                       timeout=30.0, tick=0.5):
        whole_body = self.robot.get('whole_body')
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)
        if delta is None:
            delta = 0
        while True:
            now = rospy.Time.now()
            if (now - start) > timeout:
                msg = 'frame={0}'.format(frame)
                self.fail('Timed out: {0}'.format(msg))
            pose = whole_body.get_end_effector_pose(frame)
            result_list = pose[0] + pose[1]
            goal_list = goal[0] + goal[1]
            if all(abs(a - b) < delta for a, b in zip(result_list, goal_list)):
                break
            else:
                rospy.sleep(tick)

    def assert_base_reach_goal(self, goal):
        pass

    def test_5_1_2_1(self):
        """Getting started"""
        omni_base = self.robot.get('omni_base')
        omni_base.go(0.1, 0.0, 0.0, 10.0, relative=True)
        self.assert_pose_equal(omni_base.get_pose('map'),
                               geometry.pose(0.1, 0.0, 0.0),
                               delta=0.01)

        omni_base.go(-0.1, 0.0, 0.0, 10.0, relative=True)
        self.assert_pose_equal(omni_base.get_pose('map'),
                               geometry.pose(0.0, 0.0, 0.0),
                               delta=0.01)

    def test_5_1_3_01(self):
        """Pose transition"""

        whole_body = self.robot.get('whole_body')

        whole_body.move_to_neutral()
        self.assert_joints_reach_goals(EXPECTED_NEUTRAL, 0.01)

        whole_body.move_to_go()
        self.assert_joints_reach_goals(EXPECTED_TO_GO, 0.01)

        whole_body.move_to_neutral()
        self.assert_joints_reach_goals(EXPECTED_NEUTRAL, 0.01)

    def test_5_1_3_02(self):
        """Driving joints"""
        whole_body = self.robot.get('whole_body')
        whole_body.move_to_neutral()

        self.assertListEqual(JOINT_NAMES, whole_body.joint_names)

        whole_body.move_to_joint_positions({'arm_lift_joint': 0.2})
        expected_pose = {'arm_lift_joint': 0.2}
        self.assert_joints_reach_goals(expected_pose, delta=0.01)

        whole_body.move_to_joint_positions({'head_pan_joint': 0.4,
                                            'head_tilt_joint': -0.2})
        expected_pose = {'head_pan_joint': 0.4,
                         'head_tilt_joint': -0.2}
        self.assert_joints_reach_goals(expected_pose, delta=0.01)

    def test_5_1_3_03(self):
        """Driving gripper"""
        gripper = self.robot.get('gripper')
        whole_body = self.robot.get('whole_body')
        whole_body.move_to_neutral()
        self.assert_joints_reach_goals(EXPECTED_NEUTRAL, 0.01)

        gripper.command(1.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        gripper.command(0.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

        gripper.command(1.0)
        self.assert_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        gripper.grasp(-0.01)
        self.assert_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

    def test_5_1_3_04(self):
        """Moving end-effector"""
        whole_body = self.robot.get('whole_body')
        whole_body.move_to_neutral()
        self.assert_joints_reach_goals(EXPECTED_NEUTRAL, delta=0.01)

        goal =  whole_body.get_end_effector_pose('map')
        whole_body.move_end_effector_pose(geometry.pose(z=1.0),
                                          'hand_palm_link')
        goal = ((goal[0].x + 1.0, goal[0].y, goal[0].z), goal[1])
        self.assert_end_effector_reach_goal(goal, delta=0.01)

    def test_5_1_3_05(self):
        """Coordinates and tf"""
        self.fail("Not implemented.")

    def test_5_1_3_06(self):
        """Moving end-effector with tf"""
        self.fail("Not implemented.")

    def test_5_1_3_07(self):
        """Moving end-effector by line"""
        self.fail("Not implemented.")

    def test_5_1_3_08(self):
        """Hand impedance control"""
        self.fail("Not implemented.")

    def test_5_1_3_09(self):
        """Changing arm/base ratio in motion planning"""
        self.fail("Not implemented.")

    def test_5_1_3_10(self):
        """Collision avoidance"""
        self.fail("Not implemented.")

    def test_5_1_3_11(self):
        """This section uses a real robot."""

    def test_5_1_4_1(self):
        """Get current position"""
        self.fail("Not implemented.")

    def test_5_1_4_2(self):
        """Move to global position"""
        self.fail("Not implemented.")

    def test_5_1_4_3(self):
        """Move to relative position"""
        self.fail("Not implemented.")

    def test_5_1_4_4(self):
        """Relative move using tf"""
        self.fail("Not implemented.")

    def test_5_1_6(self):
        """Marker recognition"""
        self.fail("Not implemented.")

    def test_5_1_7(self):
        """Floor navigator"""
        self.fail("Not implemented.")

    def test_5_1_8(self):
        """Grasping example"""
        self.fail("Not implemented.")

    def test_5_1_9(self):
        """LINEMOD example"""
        self.fail("Not implemented.")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('hsrb_interface_py', 'test_user_manual', TestUserManual)
