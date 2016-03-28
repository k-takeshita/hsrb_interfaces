# vim: fileencoding=utf-8
"""Provide abstract battery interface."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_funcition
from __future__ import unicode_literals

from tmc_msgs.msg import BatteryState

from . import robot
from . import settings
from . import utils


class Battery(robot.Item):
    """Abstract interface to get battery status."""

    def __init__(self, name):
        """Initialize an instance.

        Args:
            name (str): Name of an exptected resource
        """
        super(Battery, self).__init__()
        self._setting = settings.get_entry('power_supply', name)
        topic = self._setting['topic']
        self._sub = utils.CachingSubscriber(topic, BatteryState)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    @property
    def charge(self):
        """(float): Remaining battery charge [%]."""
        return self._sub.data.power

    @property
    def temperature(self):
        """(int):  Battery temperature [deg C]."""
        return self._sub.data.temperature
