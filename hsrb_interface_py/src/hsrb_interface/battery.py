#!/usr/bin/env python
# vim: fileencoding=utf-8

from . import robot
from . import settings
from . import utils

from tmc_msgs.msg import BatteryState

class Battery(robot.Resource):
    u"""バッテリー情報を取得するためのクラス

    Attributes:
        charge (float):      バッテリーチャージ残量[%]
        temperature (float): バッテリー温度[deg C]

    """
    def __init__(self, name):
        super(Battery, self).__init__()
        self._setting = settings.get_entry('power_supply', name)
        print self._setting
        topic = self._setting['topic']
        self._sub = utils.CachingSubscriber(topic, BatteryState)

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


