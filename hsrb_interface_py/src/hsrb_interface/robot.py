#!/usr/bin/env python
# vim: fileencoding=utf-8
import rospy


from .utils import CachingSubscriber, SingletonMeta
from .settings import settings
from .exceptions import RobotInitializationError


class Robot(object):
    u"""ロボットとの接続と、各デバイスのリソース管理を行うコンテキストマネージャ

    with文に対応している。

    """
    __metaclass__ = SingletonMeta

    _initialized = False

    def __init__(self):
        rospy.init_node('hsrb_interface_py', anonymous=True)
        Robot._initialized = True

    def close(self):
        u"""直ちに接続を閉じる"""
        self.__exit__(None, None, None)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if Robot._initialized == False:
            return
        if exc_type is None:
            rospy.signal_shutdown('shutdown normally')
        else:
            rospy.signal_shutdown('shutdown by an exception')
        Robot._initialized = False

    @property
    def name(self):
        return settings['robot']['name']

    def list_joint_group(self):
        return [r['name'] for r in settings['joint_group']]

    def list_mobile_base(self):
        return [r['name'] for r in settings['mobile_base']]

    def list_camera(self):
        return [r['name'] for r in settings['camera']]

    def list_laser_scan(self):
        return [r['name'] for r in settings['laser_scan']]

    def list_imu(self):
        return [r['name'] for r in settings['imu']]

    def list_force_torque(self):
        return [r['name'] for r in settings['force_torque']]

    def list_power_source(self):
        return [r['name'] for r in settings['power_source']]

    def list_end_effector(self):
        return [r['name'] for r in settings['end_effector']]

    def list_object_detector(self):
        return [r['name'] for r in settings['object_detector']]

    def list_collision_map(self):
        return [r['name'] for r in settings['collision_map']]

    def list_text_to_speech(self):
        return [r['name'] for r in settings['text_to_speech']]

class Resource(object):
    u"""
    """
    def __init__(self):
        if not Robot._initialized:
            raise RobotInitializationError("Robot is not initialized correcly")
