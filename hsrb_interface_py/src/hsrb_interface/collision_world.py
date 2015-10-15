#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy

from tmc_manipulation_msgs.srv import (
    GetCollisionEnvironment,
    GetCollisionEnvironmentRequest,
)

from tmc_manipulation_msgs.msg import (
    CollisionObject,
    CollisionObjectOperation
)

from tmc_msgs.msg import ObjectIdentifier

from . import robot
from . import settings

class CollisionWorld(robot.Item):
    u"""衝突検知用の空間

    Attributes:
        known_object_only (bool):
        ref_frame_id (str):
    """
    def __init__(self, name):
        super(CollisionWorld, self).__init__()
        self._setting = settings.get_entry('collision_world', name)
        self._known_object_only = True
        self._ref_frame_id = settings.get_frame('map')
        self._collision_object_pub = rospy.Publisher(self._setting['topic'],
                                                     CollisionObject,
                                                     queue_size=1)

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

    u"""衝突検知用の空間を更新

    Returns:
        tmc_manipulation_msgs.msg.CollisionEnvironment: 衝突検知用の空間
    """
    def update(self):
        req = GetCollisionEnvironmentRequest()
        req.known_object_only = self._known_object_only
        req.origin_frame_id = self._ref_frame_id

        service = rospy.ServiceProxy(self._setting['service'],
                                     GetCollisionEnvironment)
        res = service.call(req)
        return res.environment

    u"""衝突検知用の空間から物体を削除

    Args:
        object_id(tmc_msgs.msg.ObjectIdentifier): 物体ID
    """
    def delete(self, object_id):
        collision_object = CollisionObject()
        collision_object.id = object_id
        collision_object.operation.operation = CollisionObjectOperation.REMOVE
        self._collision_object_pub.publish(collision_object)
