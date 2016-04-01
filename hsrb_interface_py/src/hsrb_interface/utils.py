# vim: fileencoding=utf-8
"""Utility classes and functions"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import threading

import rospy

from . import exceptions


class CachingSubscriber(object):
    """Subscribe a topic and keep its latest message for given period."""

    def __init__(self, topic, msg_type, time_to_live=0.0, default=None,
                 **kwargs):
        """Initialize a instance

        Args:
            topic (str):                ROS topic name
            msg_type (rospy.Message):   ROS message type
            time_to_live (double):      Time to live of a latest message [sec]
            default (msg_type):         Default value for :py:attr:`.data`
            kwargs (Dict[str, object]): Options passed to rospy.Subscriber
        """
        self._lock = threading.Lock()
        self._time_to_live = rospy.Duration(time_to_live)
        self._latest_stamp = rospy.Time.now()
        kwargs['callback'] = self._callback
        self._default = default
        self._msg = default
        self._topic = topic
        self._msg_type = msg_type
        self._sub = rospy.Subscriber(topic, msg_type, **kwargs)

    def wait_for_message(self, timeout=None):
        """Wait for a new meesage until elapsed time exceeds ``timeout`` [sec].

        If ``timeout`` is None, a instance wait infinitely.

        Returns:
            None
        """
        try:
            rospy.client.wait_for_message(self._topic, self._msg_type, timeout)
        except rospy.ROSException as exc:
            raise exceptions.RobotConnectionError(exc)

    def _callback(self, msg):
        """Subscriber callback"""
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = rospy.Time.now()
            finally:
                self._lock.release()

    @property
    def data(self):
        """(Message Type): Latest topic value"""
        with self._lock:
            if not self._time_to_live.is_zero():
                now = rospy.Time.now()
                if (now - self._latest_stamp) > self._time_to_live:
                    self._msg = self._default
            return copy.deepcopy(self._msg)


def iterate(func, times=None):
    """Create a generator that yields result of ``func`` calls ``times``-times.

    Args:
        func (callable): a callable object repeatedly invoked
        times (int): Number of calls (Infinte if None)
    Returns:
        A generator object
    """
    if times is None:
        while True:
            yield func()
    else:
        for _ in range(times):
            yield func()
