#!/usr/bin/env python
# vim: fileencoding=utf-8

from .settings import settings
from .utils import CachingSubscriber

from tmc_msgs.msg import BatteryState

class Battery(object):
    u"""バッテリー状態を表す"""
    def __init__(self):
        topic = settings['battery_state']
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


