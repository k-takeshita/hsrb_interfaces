#!/usr/bin/env python
# vim: fileencoding=utf-8

class RobotConnectionError(Exception):
    pass

class ResourceNotFoundError(Exception):
    pass

class TrajectoryLengthError(Exception):
    pass

class TrajectoryFilterError(Exception):
    pass

class FollowTrajectoryError(Exception):
    pass

class PlannerError(Exception):
    pass

class GripperError(Exception):
    pass

class InvalidLanguageError(Exception):
    pass

class MobileBaseError(Exception):
    pass
