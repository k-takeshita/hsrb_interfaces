#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy

from tmc_manipulation_msgs.msg import CollisionEnvironment
from tmc_manipulation_msgs.srv import (
    GetCollisionEnvironment,
    GetCollisionEnvironmentRequest,
)

from .robot import Resource
from .settings import get_setting, get_frame

class CollisionMap(Resource):
    def __init__(self, name):
        super(CollisionMap, self).__init__()
        self._setting = get_setting('collision_map', name)

        self._known_object_only = True
        self._ref_frame_id = get_frame('map')
        self._environment = None

    @property
    def known_object_only(self):
        return self._known_object_only

    @known_object_only.setter
    def known_object_only(self, value):
        self._known_object_only = value

    @property
    def ref_frame_id(self):
        return self._ref_frame_id

    @ref_frame_id.setter
    def ref_frame_id(self, value):
        self._ref_frame_id = value


    @property
    def environment(self):
        if self._environment is None:
            self.update()

        return self._environment


    def update(self):
        req = GetCollisionEnvironmentRequest()
        req.known_object_only = self._known_object_only
        req.ref_frame_id = self._ref_frame_id

        service = rospy.ServiceProxy(self._setting['service'],
                                     GetCollisionEnvironment)
        res = service.call(req)
        self._environment = res.environment

