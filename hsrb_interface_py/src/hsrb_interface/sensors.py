#!/usr/bin/env python
# vim: fileencoding=utf-8
from cv_bridge import CvBridge


from sensor_msgs.msg import (
    Image,
    Imu,
    LaserScan,
)

from geometry_msgs.msg import WrenchStamped

from .robot import Resource
from .utils import CachingSubscriber
from .settings import get_setting, get_frame


class Camera(Resource):
    u"""カメラ

    Args:
        name (str): デバイス名
    """
    def __init__(self, name):
        super(Camera, self).__init__()
        self._name = name
        self._setting = get_setting('camera', name)

        self._cv_bridge = CvBridge()
        self._image_sub = CachingSubscriber(self._setting['prefix'] + '/image_raw', Image)

    def get_image(self):
        return self._cv_bridge.imgmsg_to_cv2(self._image_sub.data)


class ForceTorque(Resource):
    u"""6軸力センサー

    Args:
        name (str): デバイス名

    """
    def __init__(self, name):
        super(ForceTorque, self).__init__()
        self._setting = get_setting('force_torque', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, WrenchStamped)

    def get_wrench(self):
        u"""最新の値を取得する

        Returns:
            (fx, fy, fz, tx, ty, tz)
        """
        wrench = self._sub.data
        result = (wrench.wrench.force.x,
                  wrench.wrench.force.y,
                  wrench.wrench.force.z,
                  wrench.wrench.torque.x,
                  wrench.wrench.torque.y,
                  wrench.wrench.torque.z)
        return result


class IMU(Resource):
    u"""慣性センサーへのアクセスを提供する

    Args:
        name (str): デバイス名
    """
    def __init__(self, name):
        super(IMU, self).__init__()
        self._setting = get_setting('imu', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, Imu)

    def get_data(self):
        u"""最新の値を取得する
        """
        imu = self._sub.data
        ori = imu.orientaion
        angvel = imu.angular_velocity
        accel  = imu.linear_acceleration
        return (ori, angvel, accel)

class LaserScan(Resource):
    u"""レーザースキャナへのアクセスを提供する"""
    def __init__(self, name):
        super(LaserScan, self).__init__()
        self._setting = get_setting('laser_scan', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, LaserScan)

    def get_scan(self):
        u"""最新の値を取得する
        """
        return self._sub.data

