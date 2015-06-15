#!/usr/bin/env python
# vim: fileencoding=utf-8
import threading
import rospy
import copy
import functools

from tmc_vision_msgs.msg import RecognizedObject

from .settings import settings


def _expired(now, expiration, obj):
    return (now - obj.header.stamp) > expiration

class ObjectDetector(object):
    u"""オブジェクト認識機の結果を保持するオブジェクト

    Parameters:
        expiration (float): オブジェクトを検知してから、無効になるまでの時間[sec]
    """
    def __init__(self, expiration=10.0):
        self._lock = threading.Lock()
        self._cache = {}
        self._expiration = rospy.Duration(expiration)
        self._sub = rospy.Subscriber(settings['recognized_object_topic'],
                                     RecognizedObject,
                                     self._callback, queue_size=100)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._cache[msg.object_id.object_id] = msg
            finally:
                self._lock.release()

    def get_objects(self):
        with self._lock:
            now = rospy.Time.now()
            new_cache = dict([(k, v) for k, v in self._cache.items() if not _expired(now, self._expiration, v)])
            self._cache = new_cache
            objects = copy.deepcopy(self._cache.values())
        results = []
        for obj in objects:
            frame_id = obj.header.frame_id
            pos = (obj.object_frame.position.x,
                   obj.object_frame.position.y,
                   obj.object_frame.position.z)
            ori = (obj.object_frame.orientation.x,
                   obj.object_frame.orientation.y,
                   obj.object_frame.orientation.z,
                   obj.object_frame.orientation.w)

            result = (obj.object_id.object_id, obj.object_id.name, frame_id, pos, ori)
            results.append(result)
        return results

