# + "/hsrb/omni_base_controllere!/usr/bin/env python
# vim: fileencoding=utf-8

class RobotInitializationError(Exception):
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
