# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""This module contains classes to control end-effector."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

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
            effort (float): Force applied to grasping [?]

        Returns:
            None
        """
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        self._grasp_client.send_goal(goal)
        try:
            timeout = rospy.Duration(_GRIPPER_GRASP_TIMEOUT)
            if self._grasp_client.wait_for_result(timeout):
                self._grasp_client.get_result()
                state = self._grasp_client.get_state()
                if state != actionlib.GoalStatus.SUCCEEDED:
                    raise exceptions.GripperError("Failed to grasp")
            else:
                self._grasp_client.cancel_goal()
                raise exceptions.GripperError("Timed out")
        except KeyboardInterrupt:
            self._grasp_client.cancel_goal()


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
