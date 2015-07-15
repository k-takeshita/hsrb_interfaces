#!/usr/bin/env python
# vim: fileencoding=utf-8

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from tmc_control_msgs.msg import GripperApplyEffortAction, GripperApplyEffortGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Bool

import rospy
import actionlib
from . import robot
from . import settings
from . import utils
from . import exceptions

_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT = 20.0
_GRIPPER_GRASP_TIMEOUT = 20.0

class Gripper(robot.Resource):
    u"""HRHグリッパーの制御を行うクラス

    """
    def __init__(self, name):
        super(Gripper, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        if self._setting is None:
            raise exceptions.ResourceNotFoundError('{0}({1}) is not found '.format('end_effector', name))
        self._name = name
        self._joint_names = self._setting['joint_names']
        prefix = self._setting['prefix']
        self._follow_joint_trajectory_client = actionlib.SimpleActionClient(
                                                   prefix + "/follow_joint_trajectory",
                                                   FollowJointTrajectoryAction)
        self._grasp_client = actionlib.SimpleActionClient(prefix + "/grasp",
                                                          GripperApplyEffortAction)

    def command(self, open_distance):
        u"""グリッパーの開き量を指示する

        Args:
            open_distance (float): グリッパーの開き量[?]
        Returns:
            None
        """
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(positions=[open_angle], time_from_start=rospy.Duration(10.0))
        ]

        self._follow_joint_trajectory_client.send_goal(goal)
        self._follow_joint_trajectory_client.wait_for_result(rospy.Duration(_GRIPPER_FOLLOW_TRAJECTORY_TIMEOUT))
        s = self._follow_joint_trajectory_client.get_state()
        if s != actionlib.GoalStatus.SUCCEEDED:
            raise exceptions.GripperError("Failed to follow commanded trajectory")

    def grasp(self, effort):
        u"""グリッパーの握りこみ動作を実行する

        Args:
            effort (float): 握りこみにかける力[?]
        Returns:
            None
        """
        goal = GripperApplyEffortGoal()
        goal.effort = effort
        self._grasp_client.send_goal(goal)
        self._grasp_client.wait_for_result(rospy.Duration(_GRIPPER_GRASP_TIMEOUT))
        self._grasp_client.get_result()
        s = self._grasp_client.get_state()
        if s != actionlib.GoalStatus.SUCCEEDED:
            raise exceptions.GripperError("Failed to grasp")


class Suction(object):
    u"""吸引ノズルの制御

    Args:
        name (str): デバイス名
    Returns:
        None
    """

    def __init__(self, name):
        super(Suction, self).__init__()
        self._setting = settings.get_entry('end_effector', name)
        if self._setting is None:
            raise exceptions.ResourceNotFoundError('{0}({1}) is not found '.format('end_effector', name))
        self._name = name
        self._pub = rospy.Publisher(self._setting['suction_topic'], Bool)
        self._sub = utils.CachingSubscriber(self._setting['pressure_sensor_topic'], Bool)

    def command(self, command):
        u"""吸引ノズルのOn/Off制御

        Args:
            command (bool): 吸引ノズルのOn/Off
        Returns:
            None
        """
        msg = Bool()
        msg.data = command
        self._pub.publish(msg)

    @property
    def pressure_sensor(self):
        u"""吸引ノズル圧力センサーのOn/Off

        Returns:
            bool:
        """
        return self._sub.data.data



