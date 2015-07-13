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
from .geometry import (
    Vector3,
    Quaternion,
    from_ros_vector3,
    from_ros_quaternion
)


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


class Camera(Resource):
    u"""カメラ

    Args:
        name (str): デバイス名
    """
    def __init__(self, name):
        super(Camera, self).__init__()
        self._name = name
        self._setting = get_setting('camera', name)

        self._image_sub = CachingSubscriber(self._setting['prefix'] + '/image_raw', Image)

    def get_image(self):
        return Image(self._image_sub.data)


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
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)): 最新の取得値
        """
        wrench = self._sub.data
        result = (from_ros_vector3(wrench.force),
                  from_ros_vector3(wrench.torque))
        return result


class IMU(Resource):
    u"""慣性センサーへのアクセスを提供する

    Args:
        name (str): デバイス名
    """
    def __init__(self, *args, **kwargs):
        super(IMU, self).__init__()
        self._setting = get_setting('imu', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, Imu)

    def get_data(self):
        u"""最新の値を取得する
        """
        imu = self._sub.data
        ori = from_ros_quaternion(imu.orientaion)
        angvel = from_ros_vector(imu.angular_velocity)
        accel  = from_ros_vector(imu.linear_acceleration)
        return (ori, angvel, accel)



class Lidar(Resource):
    u"""レーザースキャナへのアクセスを提供する"""
    def __init__(self, *args, **kwargs):
        super(LaserScan, self).__init__()
        self._setting = get_setting('laser_scan', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, LaserScan)

    def get_scan(self):
        u"""最新の値を取得する
        """
        return self._sub.data


class DigitalIO(Resource):
    u"""デジタルI/O
    """
    def __init__(self, *args, **kwargs):
        super(LaserScan, self).__init__()
        self._setting = get_setting('laser_scan', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = CachingSubscriber(topic, LaserScan)



