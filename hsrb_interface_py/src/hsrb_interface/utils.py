#!/usr/bin/env python
# vim: fileencoding=utf-8
import copy
import threading

import rospy

from . import exceptions

class CachingSubscriber(object):
    u"""指定したトピックを購読し、指定時間最新の値を保持するクラス

    Args:
        topic (str):                       トピック名
        msg_type (rospy.Message):          ROSメッセージ
        time_to_live (double):             生存期間[sec]
        default (msg_type):  値が無効化した時の値
        
        kwargs (Dict[str, object]):        rospy.Subscriberのオプション
    Attributes:
        data (msg_type): 保持する最新の値
    """
    def __init__(self, topic, msg_type, time_to_live=0.0, default=None, **kwargs):
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
        try:
            rospy.client.wait_for_message(self._topic, self._msg_type, timeout)
        except rospy.ROSException as e:
            raise exceptions.RobotConnectionError(e)

    def _callback(self, msg):
        if self._lock.acquire(False):
            try:
                self._msg = msg
                self._latest_stamp = rospy.Time.now()
            finally:
                self._lock.release()

    @property
    def data(self):
        with self._lock:
            if not self._time_to_live.is_zero():
                now = rospy.Time.now()
                if (now - self._latest_stamp) > self._time_to_live:
                    self._msg = self._default
            return copy.deepcopy(self._msg)





def iterate(func, times=None):
    """funcを繰り返し呼び出した結果を返すジェネレータを返す

    Args:
        func (callable): 繰り返し呼ばれるcallableオブジェクト
        times (int): 呼び出し回数。Noneで無限。
    Returns:
        funcを繰り返し呼び出した結果を返すジェネレータオブジェクト
    """
    if times is None:
        while True:
            yield func()
    else:
        for i in range(times):
            yield func()

