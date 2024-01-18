# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Defitions of common exceptions."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from moveit_msgs.msg import MoveItErrorCodes


class HsrbInterfaceError(Exception):
    """Base class for all other exceptions"""


class RobotConnectionError(HsrbInterfaceError):
    """An error that express a failure in a connection to a robot."""


class ResourceNotFoundError(HsrbInterfaceError):
    """An error that express a specified resource does not exist."""


class TrajectoryLengthError(HsrbInterfaceError):
    """A logical error in handling trajectories."""


class FollowTrajectoryError(HsrbInterfaceError):
    """Playing back a trajectory went wrong."""


class PlannerError(HsrbInterfaceError):
    """A request to motion planner failed.

    This exception is deprecated. Use MotionPlanningError instread.
    """


class TrajectoryFilterError(HsrbInterfaceError):
    """A request to a trajectory filter service failed.

    Args:
        message (str): Error message
        error_code (MoveItErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(TrajectoryFilterError, self).__init__(message)
        self._error_code = error_code

    def __str__(self):
        error_codes = MoveItErrorCodes.__dict__.items()
        error_names = [k for k, v in error_codes
                       if v == self._error_code.val and k.isupper()]
        if len(error_names) != 0:
            text = error_names[0]
        else:
            text = str(self._error_code.val)

        msg = super(TrajectoryFilterError, self).__str__()
        return "{1} ({0})".format(msg, text)


class MotionPlanningError(PlannerError):
    """Translate a motion planning error code to a human readable text.

    Args:
        message (str): Error message
        error_code (MoveItErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(MotionPlanningError, self).__init__(message)
        self._error_code = error_code

    def __str__(self):
        error_codes = MoveItErrorCodes.__dict__.items()
        error_names = [k for k, v in error_codes
                       if v == self._error_code.val and k.isupper()]
        if len(error_names) != 0:
            text = error_names[0]
        else:
            text = str(self._error_code.val)

        msg = super(MotionPlanningError, self).__str__()
        return "{1} ({0})".format(msg, text)


class GripperError(HsrbInterfaceError):
    """A command to a gripper failed."""


class InvalidLanguageError(HsrbInterfaceError):
    """A TTS service does not accept a given language."""


class MobileBaseError(HsrbInterfaceError):
    """Something wrong in a mobile base."""


class DeprecationWarning(Warning):
    """Indicate a feature is deprecated."""
