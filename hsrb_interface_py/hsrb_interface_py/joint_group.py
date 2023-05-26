# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""This module contains classes and functions to move joints."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math
import sys

import numpy as np
from rclpy.duration import Duration
from sensor_msgs.msg import JointState
import tf_transformations as T

# from . import collision_world
from . import exceptions
from . import geometry
from . import robot
from . import settings
# from . import trajectory
from . import utils


_DEBUG = False

# Timeout for motion planning [sec]
_PLANNING_ARM_TIMEOUT = 10.0

# Max number of iteration of moition planning
_PLANNING_MAX_ITERATION = 10000

# Goal generation probability in moition planning
_PLANNING_GOAL_GENERATION = 0.3

# Goal deviation in motion planning
_PLANNING_GOAL_DEVIATION = 0.3

# Timeout to receive a tf message [sec]
_TF_TIMEOUT = 5.0


def _normalize_np(vec):
    """Normalize 1D numpy.ndarray

    Args:
        vec (numpy.ndarray): A vector to be normalized
    Returns:
        numpy.ndarray: The reuslt of computation
    """
    length = np.linalg.norm(vec)
    if length < sys.float_info.epsilon:
        return vec
    else:
        vec = vec / length
        return vec


def _pose_from_x_axis(axis):
    """Compute a transformation that fits X-axis of its frame to given vector.

    Args:
        axis (geometry.Vector3): A target vector

    Returns:
        geometry.Pose: The result transformation that stored in Pose type.
    """
    axis = np.array(axis, dtype='float64', copy=True)
    axis = _normalize_np(axis)
    unit_x = np.array([1, 0, 0])
    outerp = np.cross(unit_x, axis)
    theta = math.acos(np.dot(unit_x, axis))
    if np.linalg.norm(outerp) < sys.float_info.epsilon:
        outerp = np.array([0, 1, 0])
    outerp = _normalize_np(outerp)
    q = T.quaternion_about_axis(theta, outerp)
    return geometry.Pose(geometry.Vector3(0, 0, 0), geometry.Quaternion(*q))


def _movement_axis_and_distance(pose1, pose2):
    """Compute a vector from the origin of pose1 to pose2 and distance.

    Args:
        pose1 (geometry.Pose): A pose that its origin is used as start.
        pose2 (geometry.Pose): A pose that its origin is used as goal.
    Returns:
        Tuple[geometry.Vector3, float]: The result
    """
    p1 = pose1[0]
    p2 = pose2[0]
    """Normalize Vector3"""
    x = p2[0] - p1[0]
    y = p2[1] - p1[1]
    z = p2[2] - p1[2]
    length = math.sqrt(x * x + y * y + z * z)
    if length < sys.float_info.epsilon:
        return geometry.Vector3(0.0, 0.0, 0.0), 0.0
    else:
        x /= length
        y /= length
        z /= length
        return geometry.Vector3(x, y, z), length


def _invert_pose(pose):
    """Invert a given pose as if it is a transformation.

    Args:
        pose (geometry.Pose): A pose to be inverted.q
    Returns:
        geometry.Pose: The result of computation
    """
    m = T.compose_matrix(translate=pose[0],
                         angles=T.euler_from_quaternion(pose[1]))
    (_, _, euler, trans, _) = T.decompose_matrix(T.inverse_matrix(m))
    q = T.quaternion_from_euler(euler[0], euler[1], euler[2])
    return geometry.Pose(geometry.Vector3(*trans), geometry.Quaternion(*q))


class JointGroup(robot.Item):
    """Abstract interface to control a group of joints.

    Attributes:
        joint_names (List[str]):
            A list of available joints.
        joint_positions (Dict[str, float]):
            Current joint positions.
        joint_states (sensor_msgs.msg.JointState):
            A latest joint states.
        joint_limits (Dict[str, float]):
            Joint limits of a robot.
        collision_world (hsrb_interface_py.collsion_world.CollisionWorld):
            A present collision world to check collision.
            If None, collision checking is disabled.
        linear_weight (float):
            How much laying weight on linear movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        angular_weight (float):
            How much laying weight on angular movement of a mobile base.
            This attirbute affect a output trajectory of motion planning.
        joint_weights (dict):
            How much laying weight on each joint of robot.
            This attirbute affect a output trajectory of motion planning.
        planning_timeout (float):
            Timeout for motion planning [sec].
        impedance_config (str):
            A name of impedance control preset config.
            If None, impeance control is disabled.
            Default is None.
        use_base_timeopt (bool):
            If true, time-optimal filter is applied to a base trajectory.
        looking_hand_constraint (bool):
            If true, the robot hand is in the robot view after the execution
            of move_end_effector_*.
    """

    def __init__(self, name):
        """See class docstring."""
        super(JointGroup, self).__init__()
        self._setting = settings.get_entry('joint_group', name)
        self._position_control_clients = []
        # arm_config = self._setting['arm_controller_prefix']
        # self._position_control_clients.append(
        #     trajectory.TrajectoryController(arm_config, self._node))
        # head_config = self._setting['head_controller_prefix']
        # self._position_control_clients.append(
        #     trajectory.TrajectoryController(head_config, self._node))
        # hand_config = self._setting["hand_controller_prefix"]
        # self._position_control_clients.append(
        #     trajectory.TrajectoryController(hand_config, self._node))
        # base_config = self._setting["omni_base_controller_prefix"]
        # self._base_client = trajectory.TrajectoryController(
        #     base_config, self._node, "/base_coordinates")
        # self._position_control_clients.append(self._base_client)
        # imp_config = settings.get_entry("trajectory", "impedance_control")
        # self._impedance_client = trajectory.ImpedanceController(imp_config)
        joint_state_topic = self._setting["joint_states_topic"]
        try:
            self._joint_state_sub = utils.CachingSubscriber(
                joint_state_topic,
                JointState,
                self._node,
                default=JointState())
            timeout = self._setting.get('timeout', None)
            self._joint_state_sub.wait_for_message(timeout)
        except Exception as e:
            print("debug joint_group except", file=sys.stderr)
            raise exceptions.RobotConnectionError(e)
        self._tf2_buffer = robot._get_tf2_buffer()
        self._end_effector_frames = self._setting['end_effector_frames']
        self._end_effector_frame = self._end_effector_frames[0]
        self._passive_joints = self._setting['passive_joints']

        self._collision_world = None
        self._linear_weight = 3.0
        self._angular_weight = 1.0
        self._joint_weights = {}
        self._planning_timeout = _PLANNING_ARM_TIMEOUT
        self._use_base_timeopt = True
        self._looking_hand_constraint = False
        self._tf_timeout = _TF_TIMEOUT

        # if _DEBUG:
        #     self._vis_pub = rospy.Publisher("tsr_marker", MarkerArray,
        #                                     queue_size=1)
        #     self._tf2_pub = tf3_ros.TransformBroadcaster()

    def _get_joint_state(self):
        """Get a current joint state.

        Returns:
            sensor_msgs.JointState: Current joint state
        """
        return self._joint_state_sub.data

    @property
    def joint_names(self):
        return self._get_joint_state().name

    @property
    def joint_positions(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.position))

    @property
    def joint_velocities(self):
        joint_state = self._get_joint_state()
        return dict(zip(joint_state.name, joint_state.velocity))

    @property
    def joint_state(self):
        return self._get_joint_state()

    @property
    def end_effector_frame(self):
        """Get or set the target end effector frame of motion planning.

        This attribute affects behaviors of following methods:
        * get_end_effector_pose
        * move_end_effector_pose
        * move_end_effector_by_line
        """
        return self._end_effector_frame

    @end_effector_frame.setter
    def end_effector_frame(self, value):
        if value in set(self._end_effector_frames):
            self._end_effector_frame = value
        else:
            msg = "`ref_frame_id` must be one of end-effector frames({0})"
            raise ValueError(msg.format(self._end_effector_frames))

    @property
    def end_effector_frames(self):
        return tuple(self._end_effector_frames)

    @property
    def looking_hand_constraint(self):
        return self._looking_hand_constraint

    @looking_hand_constraint.setter
    def looking_hand_constraint(self, value):
        self._looking_hand_constraint = value

    def get_end_effector_pose(self, ref_frame_id=None):
        """Get a pose of end effector based on robot frame.

        Returns:
            Tuple[Vector3, Quaternion]
        """
        # Default reference frame is a robot frame
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('base')
        transform = self._tf2_buffer.lookup_transform(
            ref_frame_id,
            self._end_effector_frame,
            self._node.get_clock().now(),
            Duration(seconds=self._tf_timeout)
        )
        result = geometry.transform_to_tuples(transform.transform)
        return result

    def _lookup_odom_to_ref(self, ref_frame_id):
        """Resolve current reference frame transformation from ``odom``.

        Returns:
            geometry_msgs.msg.Pose:
                A transform from robot ``odom`` to ``ref_frame_id``.
        """
        odom_to_ref_ros = self._tf2_buffer.lookup_transform(
            settings.get_frame('odom'),
            ref_frame_id,
            self._node.get_clock().now(),
            Duration(self._tf_timeout)
        ).transform
        odom_to_ref_tuples = geometry.transform_to_tuples(odom_to_ref_ros)
        return geometry.tuples_to_pose(odom_to_ref_tuples)
