# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Sensor interfaces."""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

from cv_bridge import CvBridge
from geometry_msgs.msg import WrenchStamped
import numpy as np
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import Imu as ROSImu
from sensor_msgs.msg import LaserScan as ROSLaserScan
from std_srvs.srv import Empty

from . import geometry
from . import robot
from . import settings
from . import utils


class Image(object):
    """Image data object.

    Attributes:
        height (int): A number of height elements of this image.
        width (int): A number of width elements of this image.
        encoding (str): Image encoding.
    """

    def __init__(self, data):
        """Intialize an object.

        Args:
            data (sensor_msgs.msg.Image): An original image
        """
        self._data = data
        self._cv_bridge = CvBridge()

    def _height(self):
        return self._data.height
    height = property(_height)

    def _width(self):
        return self._data.width
    width = property(_width)

    def _encoding(self):
        return self._data.encoding
    encoding = property(_encoding)

    def to_ros(self):
        """Convert an internal image to sensor_msgs/Image format.

        Returns:
            sensor_msgs.msg.Image: Result of conversion.
        """
        return self._data

    def to_cv(self):
        """Convert an internal image to cv2.Mat format.

        Returns:
            cv2.Mat: Result of conversion.

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
        """Convert an internal image to numpy.ndarray format.

        Returns:
            numpy.ndarray: Result of conversion.

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
    """A (laser) scan data object."""

    def __init__(self, data):
        """Initialize with a ROS message.

        Args:
            data (sensor_msgs.msg.LaserScan): A ROS message.
        """
        self._data = data

    def to_ros(self):
        """Convert an internal scan data to sensor_msgs/LaseScan format.

        Returns:
            sensor_msgs.msg.LaserScan: Result of conversion.

        Examples:

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
        return self._data

    def to_numpy(self):
        """Convert an internal scan data to numpy.ndarray format.

        Returns:
            numpy.ndarray: Result of conversion.

        Examples:

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
        return None


class Camera(robot.Item):
    """Provide access to a camera-like device.

    Attributes:
        image (Image): A latest image data from this camera.
    """

    def __init__(self, name):
        """Initialize with a resource which has a given `name`.

        Args:
            name (str): A name of a resource.
        """
        super(Camera, self).__init__()
        self._name = name
        self._setting = settings.get_entry('camera', name)

        topic = self._setting['prefix'] + '/image_raw'
        self._sub = utils.CachingSubscriber(topic, ROSImage)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    def _get_image(self):
        return Image(self._sub.data)
    image = property(_get_image)


class ForceTorque(robot.Item):
    """Provide access to a 6-axis force-torque sensor.

    Attributes:
        wrench (Tuple[Vector3, Vector3]):
            a latest (compensated) wrench from the sensor.
                1st elment
                    forces for each axes [N].
                2nd element
                    Second vector contains torques around each axes [Nm].

        raw (Tuple[Vector3, Vector3]):
            Latest raw sensor values.

                1st element
                    contains forces for each axes [N].
                2nd element
                    contains torques around each axes [Nm].
    """

    def __init__(self, name):
        """Initialize with a given resource name.

        Args:
            name (str): A name of a device.
        """
        super(ForceTorque, self).__init__()
        self._setting = settings.get_entry('force_torque', name)
        raw_topic = self._setting['raw_topic']
        self._raw_sub = utils.CachingSubscriber(raw_topic, WrenchStamped)
        compensated_topic = self._setting['compensated_topic']
        self._compensated_sub = utils.CachingSubscriber(
            compensated_topic, WrenchStamped)

        timeout = self._setting.get('timeout', None)
        self._raw_sub.wait_for_message(timeout)
        self._compensated_sub.wait_for_message(timeout)

    def _get_wrench(self):
        """A getter for :py:attr:`wrench`."""
        wrench = self._compensated_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result
    wrench = property(_get_wrench)

    def _get_raw_wrench(self):
        """A getter for :py:attr:`raw`."""
        wrench = self._raw_sub.data
        result = (geometry.from_ros_vector3(wrench.wrench.force),
                  geometry.from_ros_vector3(wrench.wrench.torque))
        return result
    raw = property(_get_raw_wrench)

    def reset(self):
        """Reset gravity compensation offset.

        Returns:
            None
        """
        reset_service = self._node.create_client(
            Empty, self._setting['reset_service'])
        reset_service.call_async(Empty.Request())


class IMU(robot.Item):
    """Provide accces to an IMU device.

    Attributes:
        data (Tuple(Quaternion, Vector3, Vector3)):
            A latest data from this IMU.
            The data is a 3-tuple whose elements are as follows:

            1st element
                orientation
            2nd element
                angular velocity
            3rd element
                linear acceleration
    """

    def __init__(self, name):
        """Initialize with a resource which has a given `name`.

        Args:
            name (str): A name of a device.
        """
        super(IMU, self).__init__()
        self._setting = settings.get_entry('imu', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = utils.CachingSubscriber(topic, ROSImu)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    def _get_data(self):
        """A getter for :py:attr:`data`."""
        imu = self._sub.data
        ori = geometry.from_ros_quaternion(imu.orientation)
        angvel = geometry.from_ros_vector3(imu.angular_velocity)
        accel = geometry.from_ros_vector3(imu.linear_acceleration)
        return (ori, angvel, accel)
    data = property(_get_data)


class Lidar(robot.Item):
    """Provide acces to a LIDER.

    Attributes:
        scan (LaserScan): A latest scan data from this LIDAR.
    """

    def __init__(self, name):
        """Initialize with a resource which has a given `name`.

        Args:
            name (str): A name of target LIDAR resource.
        """
        super(Lidar, self).__init__()
        self._setting = settings.get_entry('lidar', name)
        topic = self._setting['topic']
        self._name = name
        self._sub = utils.CachingSubscriber(topic, ROSLaserScan)

        timeout = self._setting.get('timeout', None)
        self._sub.wait_for_message(timeout)

    def _get_scan(self):
        """Get the latest value"""
        return LaserScan(self._sub.data)
    scan = property(_get_scan)
