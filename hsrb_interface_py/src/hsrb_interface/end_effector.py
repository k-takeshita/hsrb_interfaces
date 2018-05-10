# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""This module contains classes to control end-effector."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import warnings

import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
import rospy
from std_msgs.msg import Bool
from tmc_control_msgs.msg import GripperApplyEffortAction
from tmc_control_msgs.msg import GripperApplyEffortGoal
from trajectory_msgs.msg import JointTrajectoryPoint


from . import exceptions
from . import robot
from . import settings
from . import utils

_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT = 20.0
_GRIPPER_GRASP_TIMEOUT = 20.0
_GRIPPER_APPLY_FORCE_TIMEOUT = 10.0
_GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD = 0.8
_HAND_MOMENT_ARM_LENGTH = 0.07


class Gripper(robot.Item):
    """This class controls 2-finger gripper."""

    def __init__(self, name):
        """Initialize.

        Args:
            name (str): A name of a suction device file.
        """
        super(Gripper, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        if self._setting is None:
            msg = '{0}({1}) is not found '.format('end_effector', name)
            raise exceptions.ResourceNotFoundError(msg)
        self._name = name
        self._joint_names = self._setting['joint_names']
        prefix = self._setting['prefix']
        self._follow_joint_trajectory_client = actionlib.SimpleActionClient(
            prefix + "/follow_joint_trajectory",
            FollowJointTrajectoryAction
        )
        self._grasp_client = actionlib.SimpleActionClient(
            prefix + "/grasp",
            GripperApplyEffortAction
        )
        self._apply_force_client = actionlib.SimpleActionClient(
            prefix + "/apply_force",
            GripperApplyEffortAction
        )

    def command(self, open_angle, motion_time=1.0):
        """Command open a gripper

        Args:
            open_angle (float): How much angle to open[rad]
            motion_time (float): Time to execute command[s]

        Returns:
            None

        Example:

            .. sourcecode:: python

               robot = hsrb_interface.Robot()
               gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
               gripper.command(1.2, 2.0)

        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle],
                                 time_from_start=rospy.Duration(motion_time))
        ]

        self._follow_joint_trajectory_client.send_goal(goal)
        timeout = rospy.Duration(_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT)
        try:
            if self._follow_joint_trajectory_client.wait_for_result(timeout):
                s = self._follow_joint_trajectory_client.get_state()
                if s != actionlib.GoalStatus.SUCCEEDED:
                    msg = "Failed to follow commanded trajectory"
                    raise exceptions.GripperError(msg)
            else:
                self._follow_joint_trajectory_client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            self._follow_joint_trajectory_client.cancel_goal()

    def grasp(self, effort):
        """Command a gripper to execute grasping move.

        Args:
            effort (float): Force applied to grasping [Nm]
                            The range is -1[Nm] < effort < 0[Nm]

        Returns:
            None

        Warning:
            This function is deprecated. Use :py:func:`apply_force()` instead.
        """
        msg = ' '.join(["gripper.grasp() is depreacated."
                        "Use gripper.apply_force() instead."])
        warnings.warn(msg, exceptions.DeprecationWarning)
        if effort > 0.0:
            raise exceptions.GripperError("effort shold be negative.")
        else:
            self.apply_force(-effort / _HAND_MOMENT_ARM_LENGTH)

    def apply_force(self, effort, delicate=False):
        """Command a gripper to execute applying force.

        Args:
            effort (float): Force applied to grasping [N]
                            'effort' should be positive number
            delicate (bool): Force control is on when delicate is ``True``
                             The range force control works well
                             is 0.2 [N] < effort < 0.6 [N]

        Returns:
            None
        """
        if effort < 0.0:
            msg = "negative effort is set"
            raise exceptions.GripperError(msg)
        goal = GripperApplyEffortGoal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        client = self._grasp_client
        if delicate:
            if effort < _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD:
                goal.effort = effort
                client = self._apply_force_client
            else:
                rospy.logwarn(
                    "Since effort is high, force control become invalid.")

        client.send_goal(goal)
        try:
            timeout = rospy.Duration(_GRIPPER_GRASP_TIMEOUT)
            if client.wait_for_result(timeout):
                client.get_result()
                state = client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:
                    raise exceptions.GripperError("Failed to apply force")
            else:
                client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            client.cancel_goal()


class Suction(object):
    """This class controls a suction nozzle.

    Args:
        name (str): A name of a suction device file.

    Returns:
        None
    """

    def __init__(self, name):
        """Initialize an instance with given parameters.

        Args:
            name (str):  A name of this resource
        """
        super(Suction, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        if self._setting is None:
            msg = '{0}({1}) is not found '.format('end_effector', name)
            raise exceptions.ResourceNotFoundError(msg)
        self._name = name
        pub_topic_name = self._setting['suction_topic']
        self._pub = rospy.Publisher(pub_topic_name, Bool, queue_size=0)
        sub_topic_name = self._setting['pressure_sensor_topic']
        self._sub = utils.CachingSubscriber(sub_topic_name, Bool)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    def command(self, command):
        """Command on/off to a suction-nozzle.

        Args:
            command (bool): On if command is ``True``, Off otherwise

        Returns:
            None
        """
        if command < 0:
            msg = "'{0}' is not defined.".format(command)
            raise ValueError(msg)
        msg = Bool()
        msg.data = command
        self._pub.publish(msg)

    @property
    def pressure_sensor(self):
        """Get a sensor value (On/Off) of a suction-nozzle sensor.

        Returns:
            bool: True if ON.
        """
        return self._sub.data.data
