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

from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy

from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from . import robot


class CachingSubscriber(robot.Item):
    """Subscribe a topic and keep its latest message for given period."""

    def __init__(self, topic, msg_type, time_to_live=0.0, default=None,
                 qos_profile=QoSProfile(
                     reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
                     history=QoSHistoryPolicy.SYSTEM_DEFAULT)):
        """Initialize a instance

        Args:
            topic (str):                ROS topic name
            msg_type (rclpy.Message):   ROS message type
            time_to_live (double):      Time to live of a latest message [sec]
            default (msg_type):         Default value for :py:attr:`.data`
            kwargs (Dict[str, object]): Options passed to rclpy.Subscriber
        """
        super(CachingSubscriber, self).__init__()
        self._lock = threading.Lock()
        self._time_to_live = rclpy.duration.Duration(seconds=time_to_live)
        self._latest_stamp = self._node.get_clock().now().to_msg()
        self._default = default
        self._msg = default
        self._topic = topic
        self._msg_type = msg_type
        self.message_count = 0
        self._node.create_subscription(msg_type, topic, self._callback, 1)

    def wait_for_message(self, timeout=None):
        """Wait for a new meesage until elapsed time exceeds ``timeout`` [sec].

        If ``timeout`` is None, a instance wait infinitely.

        Returns:
            None
        """
        timeout_sec = timeout
        sleep_time = 0.25
        if timeout_sec is None:
            timeout_sec = float('inf')
        self.message_count = 0
        while self._node.context.ok() and not self.message_count != 0 and timeout_sec > 0.0:
            rclpy.spin_once(self._node)
            time.sleep(sleep_time)
            timeout_sec -= sleep_time

    def _callback(self, msg):
        if len(msg.name) == 0:
            return
        self.message_count += 1
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = self._node.get_clock().now().to_msg()
            finally:
                self._lock.release()

    @property
    def data(self):
        """(Message Type): Latest topic value"""
        with self._lock:
            is_zero = self._time_to_live.nanoseconds == 0
            if not is_zero:
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


def get_parameters_from_another_node(
        node, param_service_name, parameter_names):
    client = node.create_client(GetParameters, param_service_name)
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')
    request = GetParameters.Request()
    request.names = parameter_names
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    # handle response
    response = future.result()
    if response is None:
        e = future.exception()
        raise RuntimeError(
            'Exception while calling service of node '
            "'{args.node_name}': {e}".format_map(locals()))

    return_values = []
    for pvalue in response.values:
        if pvalue.type == ParameterType.PARAMETER_BOOL:
            value = pvalue.bool_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER:
            value = pvalue.integer_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
            value = pvalue.double_value
        elif pvalue.type == ParameterType.PARAMETER_STRING:
            value = pvalue.string_value
        elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
            value = pvalue.byte_array_value
        elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
            value = pvalue.bool_array_value
        elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            value = pvalue.integer_array_value
        elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            value = pvalue.double_array_value
        elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
            value = pvalue.string_array_value
        elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
            value = None
        else:
            raise RuntimeError("Unknown parameter type '{pvalue.type}'"
                               .format_map(locals()))
        return_values.append(value)

    return return_values


def wait_until_complete(node, future, timeout=None):
    timeout_sec = timeout
    sleep_time = 0.25
    if timeout_sec is None:
        timeout_sec = float('inf')
    res = None
    while node.context.ok() and timeout_sec > 0.0:
        rclpy.spin_until_future_complete(node, future, timeout_sec=sleep_time)
        res = future.result()
        if res is not None:
            break
        timeout_sec -= sleep_time
    return res
