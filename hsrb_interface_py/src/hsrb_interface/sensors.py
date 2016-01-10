#!/usr/bin/env python
# vim: fileencoding=utf-8


from cv_bridge import CvBridge
import numpy as np
import warnings
import rospy

from sensor_msgs.msg import (
    Image as ROSImage,
    Imu,
    LaserScan as ROSLaserScan,
)

from geometry_msgs.msg import WrenchStamped

from std_srvs.srv import (
    Empty,
    EmptyRequest,
)

from . import robot
from . import utils
from . import settings
from . import geometry

class Image(object):
    u"""

    Attributes:
        height (int):
        width (int):
        encoding (str):

    """
    def __init__(self, data):
        self._data = data
        self._cv_bridge = CvBridge()

    @property
    def height(self):
        return self._data.height

    @property
    def width(self):
        return self._data.width

    @property
    def encoding(self):
        return self._data.encoding

    def to_ros(self):
        u"""ROS(sensor_msgs/Image)
        """
        return self._data

    def to_cv(self):
        u"""

        Returns:
            cv2.Mat:

        Examples:

            .. sourcecode:: python

               import hsrb_interface
               import cv2

               with hsrb_interface.Robot() as robot:
                   camera = robot.get('head_l_stereo_camera')
                   img = head_l_camera.image
                   cv2.imshow("camera", img.to_cv())
                   cv2.waitKey()

        """
        return self._cv_bridge.imgmsg_to_cv2(self._data)

    def to_numpy(self):
        u"""
        Returns:
            numpy.ndarray:

        Examples::

            .. sourcecode:: python

               import hsrb_interface
               import numpy as np
               import matplotlib.pyplot as plt

               with hsrb_interface.Robot() as robot:
                   camera = robot.get('head_l_stereo_camera')
                   img = head_l_camera.image
                   mat = img.to_numpy()
                   plt.imshow(mat)

        """
        return np.asarray(self._cv_bridge.imgmsg_to_cv2(self._data))


class LaserScan(object):
    u"""
    """
    def __init__(self, data):
        self._data = data

    def to_ros(self):
        u"""

        Examples:

            .. sourcecode:: python

               pass

        """
        return self._data

    def to_numpy(self):
        u"""

        Examples:

            .. sourcecode:: python

               pass
        """
        return None


class Camera(robot.Item):
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



class ForceTorque(robot.Item):
    u"""6軸力センサー

    Args:
        name (str): デバイス名

    """
    def __init__(self, name):
        super(ForceTorque, self).__init__()
        self._setting = settings.get_entry('force_torque', name)
        raw_topic = self._setting['raw_topic']
        self._raw_sub = utils.CachingSubscriber(raw_topic, WrenchStamped)
        compensated_topic = self._setting['compensated_topic']
        self._compensated_sub = utils.CachingSubscriber(compensated_topic,
                                                        WrenchStamped)

        timeout = self._setting.get('timeout', None)
        self._raw_sub.wait_for_message(timeout)
        self._compensated_sub.wait_for_message(timeout)

    @property
    def wrench(self):
        u"""最新の外力を取得する

        Returns:
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)): 最新の外力[N] or [Nm]
        """
        wrench = self._compensated_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result

    @property
    def raw(self):
        u"""最新の生値を取得する

        Returns:
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)): 最新の生値[N] or [Nm]
        """
        wrench = self._raw_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result

    def reset(self):
        u"""自重補償のオフセットをリセットする

        Returns:
            Bool: リセット指令の送信に成功した場合Trueを返す
        """
        reset_service = rospy.ServiceProxy(self._setting['reset_service'],
                                           Empty)
        reset_service(EmptyRequest())
        return None


class IMU(robot.Item):
    u"""慣性センサーへのアクセスを提供する

    Args:
        name (str): デバイス名

    Attributes:
        data (): pass
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



class Lidar(robot.Item):
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
