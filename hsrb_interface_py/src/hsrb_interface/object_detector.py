#!/usr/bin/env python
# vim: fileencoding=utf-8
import threading
import rospy
import copy
import functools

from tmc_vision_msgs.msg import RecognizedObject

from .robot import Resource
from .settings import get_setting


def _expired(now, expiration, obj):
    return (now - obj.header.stamp) > expiration

class Object(object):
    u"""認識された物体
    """

    def __init__(self, data):
        self._data = data


class ObjectDetector(Resource):
    u"""オブジェクト認識機の結果を保持するクラス

    Parameters:
        expiration (float): オブジェクトを検知してから、無効になるまでの時間[sec]
            デフォルトは１０秒。
    """
    def __init__(self, name, expiration=10.0):
        super(ObjectDetector, self).__init__(self)
        self._setting = get_setting('object_detector', name)
        self._lock = threading.Lock()
        self._cache = {}
        self._expiration = rospy.Duration(expiration)
        self._sub = rospy.Subscriber(self._setting['topic'],
                                     RecognizedObject,
                                     self._callback, queue_size=100)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._cache[msg.object_id.object_id] = msg
            finally:
                self._lock.release()

    def get_objects(self):
        u"""認識しているオブジェクトを返す

        Returns:
            Dict[str, Any]:
        """
        with self._lock:
            now = rospy.Time.now()
            new_cache = dict([(k, v) for k, v in self._cache.items() if not _expired(now, self._expiration, v)])
            self._cache = new_cache
            objects = copy.deepcopy(self._cache.values())

        return [Object for o in objects]


