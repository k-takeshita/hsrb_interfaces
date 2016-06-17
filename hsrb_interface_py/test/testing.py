"""Testing utilities
"""
import sys
import time
import copy
import math
import unittest

import hsrb_interface
import hsrb_interface.robot
from hsrb_interface import geometry
from geometry_msgs.msg import PoseStamped
import rospy
import tf2_ros

def dict_almost_equal(a, b, delta):
    keys = a.keys()
    if sorted(keys) == sorted(b.keys()):
        return all(abs(a[key] - b[key]) < delta for key in keys)
    else:
        raise ValueError("a and b don't share keys")


def vector3_distance(v1, v2):
    return math.sqrt(sum((a - b)**2.0 for a, b in zip(v1, v2)))


def quaternion_distance(q1, q2):
    product = sum(a * b for a, b in zip(q1, q2))
    return abs(math.acos(2.0 * product**2.0 - 1.0))


class HsrbInterfaceTest(unittest.TestCase):
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

    BASE_MOVE_TIME_TOLERANCE = 60
    BASE_MOVE_GOAL_TOLERANCE = 0.05

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
        cls.whole_body = cls.robot.get('whole_body')
        cls.omni_base = cls.robot.get('omni_base')
        cls.gripper  = cls.robot.get('gripper')
        cls.collision_world = cls.robot.get('global_collision_world')
        cls.marker = cls.robot.get('marker')
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listner = tf2_ros.TransformListener(cls.tf_buffer)

    @classmethod
    def tearDownClass(cls):
        cls.robot.close()

    def expect_pose_equal(self, pose1, pose2, delta=None):
        pos1, rot1 = pose1
        pos2, rot2 = pose2
        self.assertAlmostEqual(pos1[0], pos2[0], delta=delta)
        self.assertAlmostEqual(pos1[1], pos2[1], delta=delta)
        self.assertAlmostEqual(pos1[2], pos2[2], delta=delta)
        self.assertAlmostEqual(rot1[0], rot2[0], delta=delta)
        self.assertAlmostEqual(rot1[1], rot2[1], delta=delta)
        self.assertAlmostEqual(rot1[2], rot2[2], delta=delta)
        self.assertAlmostEqual(rot1[3], rot2[3], delta=delta)

    def expect_dict_contains_subset(self, a, b, delta=None):
        for key in a:
            if key not in b:
                self.fail()
            else:
                self.assertAlmostEqual(a[key], b[key], delta=delta)

    def wait_joints_stable(self, delta):
        last_positions = self.whole_body.joint_positions
        while True:
            positions = self.whole_body.joint_positions
            if dict_almost_equal(positions, last_positions, delta):
                break
            else:
                last_positions = copy.deepcopy(positions)
                time.sleep(0.1)

    def expect_joints_reach_goals(self, expected, delta=None,
                                  timeout=30.0, tick=0.5):
        """Success if all `expected` joints reach their goals within `timeout`.

        Args:
            expected (Dict[str, float]): Target joints and their goals.
            delta (float): Threshold to a joint reach its goal.
            timeout (float): Timeout duration in seconds.
            tick (float): Sleep duration between each check.
        """
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)
        if delta is None:
            delta = 0
        while True:
            num_reached = 0
            for joint, goal in expected.items():
                position = self.whole_body.joint_positions[joint]
                if abs(position - goal) <= delta:
                    num_reached += 1
            now = rospy.Time.now()
            if (now - start) > timeout:
                diffs = []
                positions = self.whole_body.joint_positions
                for key in expected:
                    template = '{0}: expected={1} actual={2}'
                    diff = template.format(key, expected[key], positions[key])
                    diffs.append(diff)
                self.fail('Timed out: {0}'.format(', '.join(diffs)))
            if num_reached == len(expected):
                break
            else:
                rospy.sleep(tick)

    def expect_hand_reach_goal(self, goal, frame='map',
                               pos_delta=None, ori_delta=None,
                               timeout=30.0, tick=0.5):
        """
        Args:
            goal (): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (float): Position tolerance [m]
            ori_delta (float): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)
        pos_delta = float('inf') if pos_delta is None else pos_delta
        ori_delta = float('inf') if ori_delta is None else ori_delta
        while True:
            pose = self.whole_body.get_end_effector_pose(frame)
            pos_error = vector3_distance(pose[0], goal[0])
            ori_error = quaternion_distance(pose[1], goal[1])

            if pos_error < pos_delta and ori_error < ori_delta:
                break
            else:
                rospy.sleep(tick)
            now = rospy.Time.now()
            if (now - start) > timeout:
                msg = '\n'.join([
                    'Timed out:',
                    '\nexpected  = {0}'.format(goal),
                    '\nacutal    = {0}'.format(pose),
                    '\npos_error = {0} < {1}'.format(pos_error, pos_delta),
                    '\nori_error = {0} < {1}'.format(ori_error, ori_delta),
                    '\nframe     = {0}'.format(frame),
                ])
                self.fail(msg)

    def expect_base_reach_goal(self, goal, frame='map',
                               pos_delta=None, ori_delta=None,
                               timeout=30.0, tick=0.5):
        """
        Args:
            goal (): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (float): Position tolerance [m]
            ori_delta (float): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time(0)
        goal_pose.header.frame_id = frame
        goal_pose.pose = geometry.tuples_to_pose(goal)
        map_to_goal_pose = self.tf_buffer.transform(goal_pose, 'map')
        map_to_goal_pose.pose.position.z = 0.0
        goal = geometry.pose_to_tuples(map_to_goal_pose.pose)

        pos_delta = float('inf') if pos_delta is None else pos_delta
        ori_delta = float('inf') if ori_delta is None else ori_delta
        while True:
            pose = self.omni_base.get_pose('map')
            pos_error = vector3_distance(pose[0], goal[0])
            ori_error = quaternion_distance(pose[1], goal[1])

            if pos_error < pos_delta and ori_error < ori_delta:
                break
            else:
                rospy.sleep(tick)
            now = rospy.Time.now()
            if (now - start) > timeout:
                msg = '\n'.join([
                    'Timed out:',
                    '\texpected  = {0}'.format(goal),
                    '\tacutal    = {0}'.format(pose),
                    '\tpos_error = {0} < {1}'.format(pos_error, pos_delta),
                    '\tori_error = {0} < {1}'.format(ori_error, ori_delta),
                    '\tframe     = {0}'.format(frame),
                ])
                self.fail(msg)

    def expect_object(self, object_id, expected_pose, frame='map',
                      pos_delta=None, ori_delta=None,
                      timeout=30.0, tick=0.5):
        """
        Args:
            object_id (id)
            expected_pose (): Expected goal pose
            frame (str): base frame of `goal`
            pos_delta (float): Position tolerance [m]
            ori_delta (float): Orientation tolerance
                (Measured in closest angle difference) [rad]
            timeout (float): Timeout in seconds
            tick (float): Sleep time between goal check in seconds
        """
        start = rospy.Time.now()
        timeout = rospy.Duration(timeout)

        pos_delta = float('inf') if pos_delta is None else pos_delta
        ori_delta = float('inf') if ori_delta is None else ori_delta
        while True:
            now = rospy.Time.now()
            obj = self.marker.get_object_by_id(object_id)
            if obj is not None:
                pose = obj.get_pose(frame)
                pos_error = vector3_distance(pose[0], expected_pose[0])
                ori_error = quaternion_distance(pose[1], expected_pose[1])

                if pos_error < pos_delta and ori_error < ori_delta:
                    break
            if (now - start) > timeout:
                buf = ['Timed out:']
                if obj is not None:
                    buf.extend([
                        '\texpected  = {0}'.format(expected_pose),
                        '\tactual    = {0}'.format(pose),
                        '\tpos_error = {0} < {1}'.format(pos_error, pos_delta),
                        '\tori_error = {0} < {1}'.format(ori_error, ori_delta),
                    ])
                buf.append('\tframe     = {0}'.format(frame))
                msg = '\n'.join(buf)
                self.fail(msg)
            rospy.sleep(tick)


