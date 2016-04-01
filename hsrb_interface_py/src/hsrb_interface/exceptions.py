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


class TrajectoryFilterError(HsrbInterfaceError):
    """A request to a trajectory filter service failed."""


class FollowTrajectoryError(HsrbInterfaceError):
    """Playing back a trajectory went wrong."""


class PlannerError(HsrbInterfaceError):
    """A request to motion planner failed."""


class GripperError(HsrbInterfaceError):
    """A command to a gripper failed."""


class InvalidLanguageError(HsrbInterfaceError):
    """A TTS service does not accept a given language."""


class MobileBaseError(HsrbInterfaceError):
    """Something wrong in a mobile base."""


class DeprecationWarning(Warning):
    """Indicate a feature is deprecated."""
