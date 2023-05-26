# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Defitions of common exceptions."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals


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
        error_code (ArmManipulationErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(TrajectoryFilterError, self).__init__(message)
        self._error_code = error_code


class MotionPlanningError(PlannerError):
    """Translate a motion planning error code to a human readable text.

    Args:
        message (str): Error message
        error_code (ArmManipulationErrorCodes): An error code
    """

    def __init__(self, message, error_code):
        super(MotionPlanningError, self).__init__(message)
        self._error_code = error_code


class GripperError(HsrbInterfaceError):
    """A command to a gripper failed."""


class InvalidLanguageError(HsrbInterfaceError):
    """A TTS service does not accept a given language."""


class MobileBaseError(HsrbInterfaceError):
    """Something wrong in a mobile base."""


class DeprecationWarning(Warning):
    """Indicate a feature is deprecated."""
