#!/usr/bin/env python
# vim: fileencoding=utf-8
import rospy

from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import JointState

from .joint_group import JointGroup
from .utils import CachingSubscriber, SingletonMeta
from .settings import hsrb as settings


class Robot(object):
    u"""ロボットとの接続と、各デバイスのリソース情報を保持するシングルトンクラス

    with文に対応している。
    connect/disconnectメソッド呼ぶことで、直接制御も可能

    Attributes

    """
    __metaclass__ = SingletonMeta

    def __init__(self):
        self._camera_names = (
            'head_l_stereo_camera',
            'head_r_stereo_camera',
        )
        self._rgbd_senesor_names = ('rgbd_sensor',)
        self._laser_names = ('base_scan')
        self._imu_names = ('base_imu',)
        self._force_torque_sensor_names = ('wrist_wrench',)
        self._battery_names = ('battery',)
        self._hands = ('hrh_gripper',)
        self._joint_groups = ('whole_body',)
        self._moble_base = ('omni_base',)


    def connect(self):
        u"""
        """
        rospy.init_node('hsrb_interface_py', anonymous=True)

    def disconnect(self):
        u"""シャットダウン処理"""
        rospy.signal_shutdown('shutdown from ')

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type:
            self.disconnect()
            return False
        else:
            self.disconnect()
            return True

    @property
    def cameras(self):
        return self._cameras_names

    @property
    def lasers(self):
        return self._laser_names

    @property
    def imus(self):
        return self._imu_names

    @property
    def force_torque_sensors(self):
        return self._force_torque_sensor_names

    @property
    def batteries(self):
        return self._battery_names

    @property
    def hands(self):
        return self._grippers
