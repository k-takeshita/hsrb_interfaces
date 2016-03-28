# vim: fileencoding=utf-8
"""Sensor interfaces"""

from __future__ import abosolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from cv_bridge import CvBridge
import numpy as np
import rospy

import numpy as np

from geometry_msgs.msg import WrenchStamped

from std_srvs.srv import Empty
from std_srvs.srv import EmptyRequest

from . import robot
from . import settings
from . import utils


class Image(object):
    """

    Attributes:
        height (int):
        width (int):
        encoding (str):

    """

    def __init__(self, data):
        """Intialize an object.

        Args:
            data (sensor_msgs.msg.Image): An original image
        """
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
        """Convert an internal image to sensor_msgs/Image format.

        Returns:
            sensor_msgs.msg.Image
        """
        return self._data

    def to_cv(self):
        """Convert an internal image to cv2.Mat format

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
        """Convert an internal image to numpy.ndarray format

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
    """Laser scan data object"""

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
    """Provide access to a camera-like device.

    Args:
        name (str): A name of a device
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
    """Provide access to a 6-axis force-torque sensor.

    Args:
        name (str): A name of a device
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
        """Get a latest (compensated) wrench from sensor.

        Returns:
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)):
                Latest sensor value [N] or [Nm]
        """
        wrench = self._compensated_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result

    @property
    def raw(self):
        """Get a latest raw wrench from sensor.

        Returns:
            (Vector3(fx, fy, fz), Vectro3(tx, ty, tz)):
                Latest raw sensor value [N] or [Nm]
        """
        wrench = self._raw_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result

    def reset(self):
        """Reset gravity compensation offset

        Returns:
            None
        """
        reset_service = rospy.ServiceProxy(self._setting['reset_service'],
                                           Empty)
        reset_service(EmptyRequest())
        return None


class IMU(robot.Item):
    """Provide accces to an IMU device.

    Args:
        name (str): A name of a device

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
        """Get the latest value"""
        imu = self._sub.data
        ori = geometry.from_ros_quaternion(imu.orientation)
        angvel = geometry.from_ros_vector3(imu.angular_velocity)
        accel = geometry.from_ros_vector3(imu.linear_acceleration)
        return (ori, angvel, accel)



class Lidar(robot.Item):
    """Provide acces to a LIDER.

    """

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
        """Get the latest value"""
        return LaserScan(self._sub.data)
