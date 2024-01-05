# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""This module contains classes to control end-effector."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import math
import time
import warnings

import action_msgs.msg as action_msgs
from control_msgs.action import FollowJointTrajectory
from hsrb_interface.utils import CachingSubscriber
import rclpy
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from tmc_control_msgs.action import GripperApplyEffort
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
_HAND_MOTOR_JOINT_MAX = 1.2
_HAND_MOTOR_JOINT_MIN = -0.5
_JOINT_STATE_SUB_TIMEOUT = 10.0

_DISTANCE_CONTROL_PGAIN = 0.5
_DISTANCE_CONTROL_IGAIN = 1.0
_DISTANCE_CONTROL_RATE = 10.0
_DISTANCE_CONTROL_TIME_FROM_START = 0.2
_DISTANCE_CONTROL_STALL_THRESHOLD = 0.003
_DISTANCE_CONTROL_STALL_TIMEOUT = 1.0

# TODO(OTA): 以下パラメータをurdfから取ってくる
_PALM_TO_PROXIMAL_Y = 0.0245
_PROXIMAL_TO_DISTAL_Z = 0.07
_DISTAL_JOINT_ANGLE_OFFSET = 0.087
# TODO(OTA): tip_linkがモデルから浮いているので修正する
_DISTAL_TO_TIP_Y = 0.01865
_DISTAL_TO_TIP_Z = 0.04289

_DISTANCE_MAX = (_PALM_TO_PROXIMAL_Y
                 - (_DISTAL_TO_TIP_Y * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                    + _DISTAL_TO_TIP_Z * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))
                 + _PROXIMAL_TO_DISTAL_Z * math.sin(_HAND_MOTOR_JOINT_MAX)) * 2
_DISTANCE_MIN = (_PALM_TO_PROXIMAL_Y
                 - (_DISTAL_TO_TIP_Y * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                    + _DISTAL_TO_TIP_Z * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))
                 + _PROXIMAL_TO_DISTAL_Z * math.sin(_HAND_MOTOR_JOINT_MIN)) * 2


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
        self._left_finger_joint_name = self._setting[
            'left_finger_joint_name']
        self._right_finger_joint_name = self._setting[
            'right_finger_joint_name']
        self._follow_joint_trajectory_client = ActionClient(
            self._node,
            FollowJointTrajectory,
            prefix + "/follow_joint_trajectory"
        )
        self._grasp_client = ActionClient(
            self._node,
            GripperApplyEffort,
            prefix + "/grasp"
        )

        self._apply_force_client = ActionClient(
            self._node,
            GripperApplyEffort,
            prefix + "/apply_force"
        )
        self._joint_state_sub = CachingSubscriber(
            "/joint_states",
            JointState
        )
        self._joint_state_sub.wait_for_message(
            timeout=_JOINT_STATE_SUB_TIMEOUT)

        # 直前の非同期の動作を覚えておくための変数.初期値はNone
        self._current_client = None

    def command(self, open_angle, motion_time=1.0, sync=True):
        """Command open a gripper

        Args:
            open_angle (float): How much angle to open[rad]
            motion_time (float): Time to execute command[s]
            sync (bool): Not wait the result when this arg is ``False``

        Returns:
            None

        Example:

            .. sourcecode:: python

               robot = hsrb_interface.Robot()
               gripper = robot.get('gripper', robot.Items.END_EFFECTOR)
               gripper.command(1.2, 2.0)
        """
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self._joint_names
        goal.trajectory.points = [
            JointTrajectoryPoint(
                positions=[open_angle],
                time_from_start=rclpy.duration.Duration(
                    seconds=motion_time).to_msg())]
        self._send_goal(self._follow_joint_trajectory_client, goal)
        if not sync:
            return
        start_time = self._node.get_clock().now()
        elapsed_time = rclpy.duration.Duration(seconds=0.0)
        while elapsed_time < rclpy.duration.Duration(seconds=motion_time):
            try:
                rclpy.spin_until_future_complete(
                    self._node, self._send_goal_future, timeout_sec=0.1)
                if self._send_goal_future.result() is not None:
                    state = self.get_state()
                    if (state == action_msgs.GoalStatus.STATUS_SUCCEEDED):
                        return
                    if (state != action_msgs.GoalStatus.STATUS_EXECUTING):
                        msg = "Failed to follow commanded trajectory"
                        raise exceptions.GripperError(msg)
                elapsed_time = self._node.get_clock().now() - start_time
                time.sleep(0.1)
            except KeyboardInterrupt:
                self._follow_joint_trajectory_client.cancel_goal()
        self.cancel_goal()

    def get_distance(self):
        """Command get gripper finger tip distance.

        Returns:
            double: Distance between gripper finger tips [m]
        """
        joint_state = self._joint_state_sub.data
        hand_motor_pos = joint_state.position[
            joint_state.name.index(self._joint_names[0])]
        hand_left_position = joint_state.position[
            joint_state.name.index(
                self._left_finger_joint_name)] + hand_motor_pos
        hand_right_position = joint_state.position[
            joint_state.name.index(
                self._right_finger_joint_name)] + hand_motor_pos
        return ((math.sin(hand_left_position) + math.sin(hand_right_position)) * _PROXIMAL_TO_DISTAL_Z
                + 2 * (_PALM_TO_PROXIMAL_Y
                       - (_DISTAL_TO_TIP_Y * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                          + _DISTAL_TO_TIP_Z * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))))

    def set_distance(self, distance, control_time=3.0):
        """Command set gripper finger tip distance.

        Args:
            distance (float): Distance between gripper finger tips [m]
        """
        if distance > _DISTANCE_MAX:
            open_angle = _HAND_MOTOR_JOINT_MAX
            self.command(open_angle)
        elif distance < _DISTANCE_MIN:
            open_angle = _HAND_MOTOR_JOINT_MIN
            self.command(open_angle)
        else:
            # TODO(OTA): hsrb_controller内で実装する
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = self._joint_names
            goal.trajectory.points = [
                JointTrajectoryPoint(
                    time_from_start=rclpy.duration.Duration(
                        seconds=1 / _DISTANCE_CONTROL_RATE).to_msg())]

            start_time = self._node.get_clock().now()
            elapsed_time = rclpy.duration.Duration(seconds=0.0)
            ierror = 0.0
            theta_ref = math.asin(
                ((distance / 2
                  - (_PALM_TO_PROXIMAL_Y
                     - (_DISTAL_TO_TIP_Y * math.cos(_DISTAL_JOINT_ANGLE_OFFSET)
                        + _DISTAL_TO_TIP_Z * math.sin(_DISTAL_JOINT_ANGLE_OFFSET))))
                 / _PROXIMAL_TO_DISTAL_Z))
            last_movement_time = self._node.get_clock().now()
            while elapsed_time < rclpy.duration.Duration(seconds=control_time):
                try:
                    error = distance - self.get_distance()
                    if abs(error) > _DISTANCE_CONTROL_STALL_THRESHOLD:
                        last_movement_time = self._node.get_clock().now()
                    if ((self._node.get_clock().now() - last_movement_time)
                            > rclpy.duration.Duration(seconds=_DISTANCE_CONTROL_STALL_TIMEOUT)):
                        break
                    ierror += error
                    open_angle = (theta_ref + _DISTANCE_CONTROL_PGAIN * error + _DISTANCE_CONTROL_IGAIN * ierror)
                    goal.trajectory.points = [
                        JointTrajectoryPoint(
                            positions=[open_angle],
                            time_from_start=rclpy.duration.Duration(
                                seconds=_DISTANCE_CONTROL_TIME_FROM_START).to_msg())]
                    self._follow_joint_trajectory_client.send_goal_async(goal)
                    elapsed_time = self._node.get_clock().now() - start_time
                except KeyboardInterrupt:
                    self._follow_joint_trajectory_client.cancel_goal_async()
                    return
                time.sleep(1.0 / _DISTANCE_CONTROL_RATE)

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

    def apply_force(self, effort, delicate=False, sync=True):
        """Command a gripper to execute applying force.

        Args:
            effort (float): Force applied to grasping [N]
                            'effort' should be positive number
            delicate (bool): Force control is on when delicate is ``True``
                             The range force control works well
                             is 0.2 [N] < effort < 0.6 [N]
            sync (bool): Not wait the result when this arg is ``False``

        Returns:
            None
        """
        if effort < 0.0:
            msg = "negative effort is set"
            raise exceptions.GripperError(msg)
        goal = GripperApplyEffort.Goal()
        goal.effort = - effort * _HAND_MOMENT_ARM_LENGTH
        client = self._grasp_client
        if delicate:
            if effort < _GRIPPER_APPLY_FORCE_DELICATE_THRESHOLD:
                goal.effort = effort
                client = self._apply_force_client
            else:
                self._node.get_logger().warn("Since effort is high, force control become invalid.")

        self._send_goal(client, goal)
        if not sync:
            return
        while True:
            try:
                rclpy.spin_until_future_complete(
                    self._node, self._send_goal_future, timeout_sec=0.1)
                if self._send_goal_future.result() is not None:
                    state = self.get_state()
                    if (state == action_msgs.GoalStatus.STATUS_SUCCEEDED):
                        break
                    if (state != action_msgs.GoalStatus.STATUS_EXECUTING):
                        msg = '"Failed to apply force {0}'.format(state)
                        raise exceptions.GripperError(msg)
            except KeyboardInterrupt:
                client.cancel_goal()

    def _send_goal(self, client, goal):
        self._send_goal_future = client.send_goal_async(goal)
        self._current_client = client

    def _check_state(self, goal_status):
        if self._current_client is None:
            return False
        else:
            future = self._current_client.get_result()
            return future.result().status == goal_status

    def get_state(self):
        """Get a status of the action client"""
        goal_handle = self._send_goal_future.result()
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, get_result_future, timeout_sec=0.1)
        res = get_result_future.result()
        if res is None:
            return action_msgs.GoalStatus.STATUS_EXECUTING
        else:
            return res.status

    def is_moving(self):
        """Get the state as if the robot is moving.

        Returns:
            bool: True if the robot is moving
        """
        return self._check_state(action_msgs.GoalStatus.STATUS_EXECUTING)

    def is_succeeded(self):
        """Get the state as if the robot moving was succeeded.

        Returns:
            bool: True if success
        """
        return self._check_state(action_msgs.GoalStatus.STATUS_SUCCEEDED)

    def cancel_goal(self):
        """Cancel moving."""
        if not self.is_moving():
            return
        self._current_client.cancel_goal()


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
        self._pub = self._node.create_publisher(Bool, pub_topic_name, 0)
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
