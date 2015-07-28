#!/usr/bin/env python
# vim: fileencoding=utf-8

import rospy

from cv_bridge import CvBridge


from sensor_msgs.msg import (
    Image as ROSImage,
    Imu,
    LaserScan as ROSLaserScan,
)

from geometry_msgs.msg import WrenchStamped

from . import robot
from . import utils
from . import settings
from . import geometry
from . import exceptions

class Image(object):
    u"""
    """
    def __init__(self, data):
        self._data = data

    def to_cv(self):
        cv_bridge = CvBridge()
        return self._cv_bridge.imgmsg_to_cv2(self.data)

    def to_ros(self):
        return self._data


class LaserScan(object):
    u"""
    """
    def __init__(self, data):
        self._data = data

    def to_ros(self):
        return self._data


class Camera(robot.Resource):
    u"""カメラ

    Args:
        name (str): デバイス名
    """
    def __init__(self, name):
        super(Camera, self).__init__()
        self._name = name
        self._setting = settings.get_entry('camera', name)

        topic = self._setting['prefix'] + '/image_raw'
        self._sub = utils.CachingSubscriber(topic, ROSImage)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    @property
    def image(self):
        return Image(self._sub.data)


class ForceTorque(robot.Resource):
    u"""6軸力センサー

    Args:
        name (str): デバイス名

    """
    def __init__(self, name):
        super(ForceTorque, self).__init__()
        self._setting = settings.get_entry('force_torque', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = utils.CachingSubscriber(topic, WrenchStamped)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    @property
    def wrench(self):
        u"""最新の値を取得する

        Returns:
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)): 最新の取得値
        """
        wrench = self._sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result


class IMU(robot.Resource):
    u"""慣性センサーへのアクセスを提供する

    Args:
        name (str): デバイス名
    """
    def __init__(self, name):
        super(IMU, self).__init__()
        self._setting = settings.get_entry('imu', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = utils.CachingSubscriber(topic, Imu)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    @property
    def data(self):
        u"""最新の値を取得する
        """
        imu = self._sub.data
        ori = geometry.from_ros_quaternion(imu.orientation)
        angvel = geometry.from_ros_vector3(imu.angular_velocity)
        accel  = geometry.from_ros_vector3(imu.linear_acceleration)
        return (ori, angvel, accel)



class Lidar(robot.Resource):
    u"""レーザースキャナへのアクセスを提供する"""
    def __init__(self, name):
        super(Lidar, self).__init__()
        self._setting = settings.get_entry('lidar', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = utils.CachingSubscriber(topic, ROSLaserScan)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    @property
    def scan(self):
        u"""最新の値を取得する"""
        return LaserScan(self._sub.data)


