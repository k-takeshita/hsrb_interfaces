# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Utility classes and functions"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import copy
import threading
import time

import rclpy
from rclpy.duration import Duration

from . import exceptions


class CachingSubscriber(object):
    """Subscribe a topic and keep its latest message for given period."""

    def __init__(self, topic, msg_type, node, time_to_live=0.0, default=None,
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
        self._node = node
        self._time_to_live = Duration(seconds=time_to_live)
        self._latest_stamp = self._node.get_clock().now()
        self._default = default
        self._msg = default
        self._topic = topic
        self._msg_type = msg_type
        self._sub = self._node.create_subscription(
            msg_type, topic, self._callback, 1, **kwargs)

    def wait_for_message(self, timeout=None):
        """Wait for a new meesage until elapsed time exceeds ``timeout`` [sec].

        If ``timeout`` is None, a instance wait infinitely.

        Returns:
            None
        """
        self._msg = self._default
        if timeout is not None:
            timeout_t = time.time() + timeout
            while rclpy.ok() and self._msg == self._default:
                time.sleep(0.01)
                if time.time() >= timeout_t:
                    raise exceptions.RobotConnectionError(
                        "timeout exceeded while waiting for message "
                        "on topic %s" % self._topic)
        else:
            while rclpy.ok() and self._msg == self._default:
                time.sleep(0.01)

    def _callback(self, msg):
        """Subscriber callback"""
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = self._node.get_clock().now()
            finally:
                self._lock.release()

    @property
    def data(self):
        """(Message Type): Latest topic value"""
        with self._lock:
            if self._time_to_live.nanoseconds != 0:
                now = self._node.get_clock().now()
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
