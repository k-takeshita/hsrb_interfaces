# vim: fileencoding=utf-8
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import rospy
import threading

from tmc_vision_msgs.msg import RecognizedObject

from . import geometry
from . import robot
from . import settings

_TF_TIMEOUT = 1.0


def _expired(now, expiration, stamped):
    """Check a given object is outdated or not.

    Args:
        now (rospy.Time):
            A current time stamp
        expiration (rospy.Duration):
            The time to live
        stamped (Stamped Message Type):
            A ROS message type that has a header(stamped)
    Returns:
        bool: True if the object is outdated.
    """
    return (now - stanmped.header.stamp) > expiration


class KnownObject(object):
    """Abstract representation of a recognized object"""

    def __init__(self, data):
        """Initialize from a ROS message

        Args:
            data (tmc_vision_msgs.msg.RecognizedObject):
                A ROS message of a recognized object
        """
        self._data = data

    def to_ros(self):
        """Convert ot ROS message type
        """
        return copy.deepcopy(self._data)

    def __repr__(self):
        return "<{0}: id={1} name={2}>".format(self.__class__.__name__,
                                               self._data.object_id.object_id,
                                               self._data.object_id.name)

    def __eq__(self, other):
        return self._data.object_id == other._data.object_id

    def get_pose(self, ref_frame_id=None):
        """
        """
        if ref_frame_id is None:
            ref_frame_id = settings.get_frame('map')
        msg = self._data

        camera_to_marker_pose = msg.object_frame
        tf2_buffer = robot._get_tf2_buffer()
        ref_to_camera_tf = tf2_buffer.lookup_transform(
            ref_frame_id,
            msg.header.frame_id,
            msg.header.stamp,
            rospy.Duration(_TF_TIMEOUT))
        return geometry.multiply_tuples(
            geometry.transform_to_tuples(ref_to_camera_tf.transform),
            geometry.pose_to_tuples(camera_to_marker_pose))

    @property
    def id(self):
        """A ID of this object"""
        return self._data.object_id.object_id

Object = KnownObject


class ObjectDetector(robot.Item):
    """This object collect and hold results of object detection.

    Examples:

        .. sourcecode:: python

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
        """Duration to hold newly detected objects [sec]"""
        return self._expiration.to_sec()

    @expiration.setter
    def expiration(self, value):
        """Set a value of :py:attr:`expiration` ."""
        self._expiration = rospy.Duration(value)

    def get_objects(self):
        """Get all objects currently detecting.

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

    def get_object_by_id(self, id=None):
        """Get a object from current detection pool which has given ID.

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
