#!/usr/bin/env python
# vim: fileencoding=utf-8

from .robot import Resource
from .settings import get_setting
from .utils import CachingSubscriber

from tmc_msgs.msg import BatteryState

class Battery(Resource):
    u"""バッテリー情報を取得するためのクラス
    """
    def __init__(self, name):
        super(Battery, self).__init__()
        self._setting = get_setting('power_source', name)
        topic = self._setting['topic']
        self._sub = CachingSubscriber(topic, BatteryState)

    @property
    def charge(self):
        u"""充電残量(0.0 to 100.0)

        Returns:
            float: hoge
        """
        return self._sub.data.power

    @property
    def temperature(self):
        u"""バッテリーの温度[deg C]

        Returns:
            int: hoge
        """
        return self._sub.data.temperature


