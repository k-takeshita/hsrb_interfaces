#!/usr/bin/env python
# vim: fileencoding=utf-8
from __future__ import absolute_import
import threading
import rospy
import copy

from tmc_vision_msgs.msg import RecognizedObject

from . import robot
from . import geometry
from . import settings

_TF_TIMEOUT = 1.0

def _expired(now, expiration, obj):
    return (now - obj.header.stamp) > expiration

class Object(object):
    u"""認識された物体"""

    def __init__(self, data):
        self._data = data

    def to_ros(self):
        u"""
        """
        return copy.deepcopy(self._data)

    def __repr__(self):
        return "<{0}: id={1} name={2}>".format(self.__class__.__name__,
                                               self._data.object_id.object_id,
                                               self._data.object_id.name)
    def __eq__(self, other):
        return self._data.object_id == other._data.object_id

    def get_pose(self, ref_frame_id=None):
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')
        tf2_buffer = robot._get_tf2_buffer()
        ref_to_marker_tf = tf2_buffer.lookup_transform(ref_frame_id,
			                               self._data.header.frame_id,
			                               rospy.Time(0),
			                               rospy.Duration(_TF_TIMEOUT))
       
        ref_to_marker = geometry.transform_to_tuples(ref_to_marker_tf.transform)
        marker_to_object_pose = geometry.pose_to_tuples(self._data.object_frame)
        ref_to_object_pose = geometry.multiply_tuples(ref_to_marker, marker_to_object_pose)
	return ref_to_object_pose

    @property
    def id(self):
        return self._data.object_id.object_id


class ObjectDetector(robot.Resource):
    u"""オブジェクト認識機の結果を保持するクラス

    Attributes:
        expiration (float): オブジェクトを検知してから、無効になるまでの時間[sec]
            デフォルトは１０秒。

    Examples:

        Usage::
            with Robot() as robot:
                detector = robot.get("marker", Items.OBJECT_DETECTION)
                objects = detector.get_objects()

    """
    def __init__(self, name):
        super(ObjectDetector, self).__init__()
        self._setting = settings.get_entry('object_detection', name)
        self._lock = threading.Lock()
        self._cache = {}
        self._expiration = rospy.Duration(10.0)
        self._sub = rospy.Subscriber(self._setting['topic'],
                                     RecognizedObject,
                                     self._callback, queue_size=100)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._cache[msg.object_id.object_id] = msg
            finally:
                self._lock.release()

    @property
    def expiration(self):
        return self._expiration.to_sec()

    @expiration.setter
    def expiration(self, value):
        self._expiration = rospy.Duration(value)

    def get_objects(self):
        u"""認識しているオブジェクトを返す

        Returns:
            Dict[str, Any]:
        """
        with self._lock:
            now = rospy.Time.now()
            new_cache = dict([(k, v) for k, v in self._cache.items()
                              if not _expired(now, self._expiration, v)])
            self._cache = new_cache
            objects = copy.deepcopy(self._cache.values())

        return [Object(o) for o in objects]


