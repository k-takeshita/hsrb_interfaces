# vim: fileencoding=utf-8
"""Provide abstract inteface for a mobile base."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math

import actionlib

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal

import rospy
import tf

from . import exceptions
from . import geometry
from . import robot
from . import settings

# Timeout to receve enough tf transform [sec]
_TF_TIMEOUT = 1.0

_ACTION_WAIT_TIMEOUT = 30.0


def _validate_timeout(timeout):
    """Validate a given timeout value is meaning time value."""
    if timeout < 0.0 or math.isnan(timeout) or math.isinf(timeout):
        raise ValueError("Invalid timeout: {0}".format(timeout))


class MobileBase(robot.Item):
    """Abstract interface to control a mobile base.

    Example:

        .. sourcecode:: python

           with hsrb_interface.Robot() as robot:
               omni_base = robot.get('omni_base')
               omni_base.go(1, 2, math.pi / 2.0)
               print(omni_base.pose)
    """

    def __init__(self, name):
        """Initialize an instance with a resource which has given `name`.

        Args:
            name (str): A name of a target resource.
        """
        super(MobileBase, self).__init__()
        self._setting = settings.get_entry('mobile_base', name)

        self._tf2_buffer = robot._get_tf2_buffer()

        action_name = self._setting['move_base_action']
        self._action_client = actionlib.SimpleActionClient(action_name,
                                                           MoveBaseAction)
        self._action_client.wait_for_server(
            rospy.Duration(_ACTION_WAIT_TIMEOUT))

    def move(self, pose, timeout=0.0, ref_frame_id=None):
        """Move to a specified pose.

        Args:
            pose (Tuple[Vector3, Quaternion]):
                A pose from a ``ref_frame_id`` coordinate
            timeout (float): Timeout to movement [sec].
                If not specified, deafult is 0 and wait forever.
            ref_frame_id (str):
                A reference frame of a goal. Default is ``map`` frame.

        Examples:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.get('omni_base', robot.Items.MOBILE_BASE)
                   pose = (Vector3(0.1, 0.2, 0.0), Quaternion())
                   base.move(pose, 10.0, 'base_footprint')
        """
        _validate_timeout(timeout)

        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        target_pose = PoseStamped()
        target_pose.header.frame_id = ref_frame_id
        target_pose.header.stamp = rospy.Time(0)
        target_pose.pose = geometry.tuples_to_pose(pose)
        goal = MoveBaseGoal()
        goal.target_pose = target_pose
        self._action_client.send_goal(goal)

        try:
            if self._action_client.wait_for_result(rospy.Duration(timeout)):
                state = self._action_client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:
                    error_text = self._action_client.get_goal_status_text()
                    msg = 'Failed to reach goal ({0})'.format(error_text)
                    raise exceptions.MobileBaseError(msg)
            else:
                self._action_client.cancel_goal()
                raise exceptions.MobileBaseError('Timed out')
        except KeyboardInterrupt:
            self._action_client.cancel_goal()

    def go(self, x, y, yaw, timeout=0.0, relative=False):
        """Move base to a specified pose.

        Args:
            x   (float): X-axis position on ``map`` frame [m]
            y   (float): Y-axis position on ``map`` frame [m]
            yaw (float): Yaw position on ``map`` frame [rad]
            timeout (float): Timeout until movement finish [sec].
                Default is 0.0 and wait forever.
            relative (bool): If ``True``, a robot move on robot frame.
                Otherwise a robot move on ``map`` frame.

        Raises:
            ValueError: `timeout` < 0.

        Example:

            .. sourcecode:: python

               with hsrb_interface.Robot() as robot:
                   base = robot.get('omni_base', robot.Items.MOBILE_BASE)
                   base.go(0.1, 0.2, 0.0, 10.0)
        """
        _validate_timeout(timeout)

        position = geometry.Vector3(x, y, 0)
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        orientation = geometry.Quaternion(*quat)
        pose = (position, orientation)

        if relative:
            ref_frame_id = settings.get_frame('base')
        else:
            ref_frame_id = settings.get_frame('map')
        self.move(pose, timeout, ref_frame_id)

    @property
    def pose(self):
        """Estimated pose of a robot on ``map`` frame.

        Returns:
            List[float]: A pose value structured as (x[m], y[m], yaw[rad]).
        """
        pos, ori = self.get_pose()
        quat = [ori.x, ori.y, ori.z, ori.w]
        euler_angles = tf.transformations.euler_from_quaternion(quat)
        yaw = euler_angles[2]
        return [pos.x, pos.y, yaw]

    def get_pose(self, ref_frame_id=None):
        """Get estimated pose of a robot on ``ref_frame_id`` frame.

        Args:
             ref_frame_id (str):
                 A reference frame of estimated pose. (Default ``map`` frame)
        Returns:
             Tuple[Vector3, Quaternion]:
                 A pose of ``base_footprint`` frame from ``ref_frame_id``.
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')

        trans = self._tf2_buffer.lookup_transform(ref_frame_id,
                                                  settings.get_frame('base'),
                                                  rospy.Time(0),
                                                  rospy.Duration(_TF_TIMEOUT))
        return geometry.transform_to_tuples(trans.transform)

    def _cancel(self):
        """Cancel autonomous driving."""
        self._action_client.cancel_goal()
        raise exceptions.MobileBaseError('move_base was canceled from client')
