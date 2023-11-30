# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Provides object detection elements.

Copyright (c) 2016 Toyota Motor Corporation.
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import threading

import rclpy
from tmc_vision_msgs.msg import RecognizedObject

from . import geometry
from . import robot
from . import settings

_TF_TIMEOUT = 5.0


def _expired(now, expiration, stamped):
    """Check a given object is outdated or not.

    Args:
        now (rclpy.Time):
            A current time stamp
        expiration (rclpy.Duration):
            The time to live
        stamped (Stamped Message Type):
            A ROS message type that has a header(stamped)
    Returns:
        bool: True if the object is outdated.
    """
    return (now - stamped.header.stamp) > expiration


class Object(object):
    """Abstract representation of a recognized object.

    Attributes:
        id (int): A ID of this object
    """

    def __init__(self, data):
        """Initialize from a ROS message

        Args:
            data (tmc_vision_msgs.msg.RecognizedObject):
                A ROS message of a recognized object
        """
        self._data = data

    def to_ros(self):
        """Convert ot ROS message type

        Returns:
            tmc_vision_msgs.msg.RecognizedObject: A ROS message reresentation.
        """
        return copy.deepcopy(self._data)

    def __repr__(self):
        """Human friendly object representation."""
        return "<{0}: id={1} name={2}>".format(self.__class__.__name__,
                                               self._data.object_id.object_id,
                                               self._data.object_id.name)

    def __eq__(self, other):
        """Equality is checked by only ``object_id`` ."""
        return self._data.object_id == other._data.object_id

    def get_pose(self, ref_frame_id=None):
        """Get a pose of this object based onf `ref_frame_id`.

        Args:
            ref_frame_id (str): A reference frame of a pose to be returned.

        Returns:
            Tuple[Vector3, Quaternion]: A pose of this object.
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')
        msg = self._data

        camera_to_marker_pose = msg.object_frame
        tf2_buffer = robot._get_tf2_buffer()
        dur = rclpy.duration.Duration(seconds=_TF_TIMEOUT)       
        ref_to_camera_tf = tf2_buffer.lookup_transform(
            ref_frame_id,
            msg.header.frame_id,
            msg.header.stamp,
            dur)
        return geometry.multiply_tuples(
            geometry.transform_to_tuples(ref_to_camera_tf.transform),
            geometry.pose_to_tuples(camera_to_marker_pose))

    def _get_id(self):
        return self._data.object_id.object_id
    id = property(_get_id)


class ObjectDetector(robot.Item):
    """This object collect and hold results of object detection.

    Attributes:
        expiration (float): Duration to hold newly detected objects [sec].

    Examples:
        .. sourcecode:: python

           with Robot() as robot:
               detector = robot.get("marker", Items.OBJECT_DETECTION)
               objects = detector.get_objects()

    """

    def __init__(self, name, node):
        """Initialize with a resource of a given `name`.

        Notes:
            This class is not intended to be created by user directly.

        Args:
            name (str): A name of a resource.
        """
        super(ObjectDetector, self).__init__()
        self._setting = settings.get_entry('object_detection', name)
        self._lock = threading.Lock()
        self._cache = {}
        self._expiration = rclpy.duration.Duration(seconds=10.0)
        self._sub = node._conn.create_subscription(RecognizedObject, self._setting['topic'], self._callback,100)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._cache[msg.object_id.object_id] = msg
            finally:
                self._lock.release()

    def _get_expiration(self):
        return self._expiration.to_sec()

    def _set_expiration(self, value):
        self._expiration = rclpy.duration.Duration(seconds=value)

    expiration = property(_get_expiration, _set_expiration,
                          "Duration to hold newly detected objects [sec]")

    def get_objects(self):
        """Get all objects currently detecting.

        Returns:
            Dict[str, Any]:
        """
        with self._lock:
            now = self.get_clock().now()
            new_cache = dict([(k, v) for k, v in self._cache.items()
                              if not _expired(now, self._expiration, v)])
            self._cache = new_cache
            objects = copy.deepcopy(self._cache.values())

        return [Object(o) for o in objects]

    def get_object_by_id(self, id=None):
        """Get a object from current detection pool which has given ID.

        Args:
            id (int): if `id` is  ``None`` .

        Returns:
            A recognized object.
            If ID is not found in the pool, it returns None.
        """
        objects = self.get_objects()
        filtered = [obj for obj in objects if obj.id == id]
        if filtered:
            return filtered[0]
        else:
            return None
