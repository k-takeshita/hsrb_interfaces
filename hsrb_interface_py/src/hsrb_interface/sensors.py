#!/usr/bin/env python
# vim: fileencoding=utf-8
from cv_bridge import CvBridge


from sensor_msgs.msg import (
    Image,
    PointCloud2,
    Imu,
    JointState,
    LaserScan,
)

from geometry_msgs.msg import WrenchStamped

from .core import NoSuchDeviceError
from .utils import CachingSubscriber
from .settings import settings

class Camera(object):
    u"""カメラ

    Args:
        name (str): デバイス名
    """
    def __init__(self, name, prefix):
        self._name = name
        self._cv_bridge = CvBridge()
        self._image_sub = CachingSubscriber(prefix + '/image_raw', Image)

    def get_image(self):
        return self._cv_bridge.imgmsg_to_cv2(self._image_sub.data)


class RGBDSensor(object):
    u"""
    Args:
        name (str): デバイス名
    """
    def __init__(self, name, prefix):
        self._name = name
        self._cv_bridge = CvBridge()
        self._image_sub = CachingSubscriber(prefix + '/rgb/image_raw', Image)
        self._depth_sub = CachingSubscriber(prefix + '/depth/image_raw', Image)

    def get_image(self):
        u"""
        """
        return self._cv_bridge.imgmsg_to_cv2(self._image_sub.data)

    def get_depth(self):
        u"""
        """
        return self._cv_bridge.imgmsg_to_cv2(self._depth_sub.data)


class ForceTorqueSensor(object):
    u"""6軸力センサーへのアクセスを提供する

    Args:
        name (str): デバイス名

    Attributes:
        data ()

    """
    def __init__(self, name, topic):
        self._name = name
        self._sub = CachingSubscriber(topic, WrenchStamped)

    def get_value(self):
        return self._sub.data

class IMU(object):
    u"""慣性センサーへのアクセスを提供する

    Args:
        name (str): デバイス名
    """
    def __init__(self, name, topic):
        self._name = name
        self._sub = CachingSubscriber(topic, Imu)

    def get_value(self):
        return self._sub.data

class LaserScanner(object):
    u"""レーザースキャナへのアクセスを提供する"""
    def __init__(self, name, topic):
        self._name = name
        self._sub = CachingSubscriber(topic, LaserScan)

    @property
    def data(self):
        return self._sub.data
