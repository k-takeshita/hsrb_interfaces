#!/usr/bin/env python
# vim: fileencoding=utf-8
#
# Copyright (c) 2015, TOYOTA MOTOR CORPORATION
# All rights reserved.
#
from __future__ import absolute_import
import weakref
import rospy
import importlib
import enum

from . import utils
from . import settings
from . import exceptions


class Resource(object):
    u"""リソース管理を受けるオブジェクトのベースクラス"""
    def __init__(self):
        if not Robot.connecting():
            raise exceptions.RobotConnectionError("No robot connection")



class _ConnectionManager(object):
    u"""ロボットとの接続制御を行うオブジェクト"""

    def __init__(self):
        rospy.init_node('hsrb_interface_py', anonymous=True)
        self._registry = {}

    def __del__(self):
        rospy.signal_shutdown('shutdown')

    def list(self, res_type):
        u"""
        """
        if res_type is None:
            targets = [x.value for x in  Robot.Items]
        else:
            targets = [res_type]
        results = []
        for target in targets:
            section = settings.get_section(target)
            if section is None:
                raise exceptions.ResourceNotFoundError("No such category ({0})".format(target))
            for key in section.keys():
                results.append((target, key))
        return results

    def get(self, name, res_type):
        u"""
        """
        key = (name, res_type)
        if key in self._registry:
            return self._registry[key]
        else:
            config = settings.get_entry(res_type.value, name)
            module_name, class_name= config["class"]
            module = importlib.import_module(".{0}".format(module_name), "hsrb_interface")
            cls = getattr(module, class_name)
            obj = cls(name)
            self._registry[key] = obj
            return weakref.proxy(obj)


class Robot(object):
    u"""ロボットとの接続を管理するハンドル

    複数のインスタンスを作ることができ、その場合最後のインスタンスで`close`が
    呼び出されるか、すべてのインスタンスが破壊されると接続が切断される。

    新たに接続を確立するには、新しいインスタンスを生成する必要がある。

    Example:

        with Robot() as r:
            print r.list(Robot.Items.JOINT_GROUP)
            j = r.get("whole_body", Robot.Items.JOINT_GROUP)

    Attributes:
        name (str): ロボット名
    """

    class Items(enum.Enum):
        JOINT_GROUP     = 'joint_group'
        MOBILE_BASE     = 'mobile_base'
        END_EFFECTOR    = 'end_effector'
        CAMERA          = 'camera'
        FORCE_TORQUE    = 'force_torque'
        IMU             = 'imu'
        LIDAR           = 'lidar'
        BATTERY         = 'power_supply'
        OBJECT_DETECTOR = 'object_detector'
        COLLISION_WORLD = 'collision_world'
        TEXT_TO_SPEECH  = 'text_to_speech'

    _connection = None

    @staticmethod
    def connecting():
        return Robot._connection is not None and Robot._connection() is not None

    def __init__(self, *args, **kwargs):
        if Robot._connection is None or Robot._connection() is None:
            self._conn = _ConnectionManager()
            Robot._connection = weakref.ref(self._conn)
        else:
            self._conn = Robot._connection()

    def close(self):
        u"""直ちに接続を閉じる

        """
        self.__exit__(None, None, None)

    def __enter__(self):
        u"""ContextManagerインターフェースの一部"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        u"""ContextManagerインターフェースの一部"""
        self._conn = None

    def ok(self):
        u"""初期化されていれば``True``"""
        return self._conn is not None

    @property
    def name(self):
        u"""ロボット名を返す
        """
        return settings.get_entry('robot', 'hsrb')['fullname']

    def list(self, res_type=None):
        u"""利用可能なResourceをリストアップする。

        Returns:
            List[str]: 利用可能なリスト
        """
        return self._conn.list(res_type)


    def get(self, name, res_type=None):
        u"""
        Returns:

        Raises:
            ResourceNotFoundError
        """
        return self._conn.get(name, res_type)
