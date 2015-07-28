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
import tf2_ros

from . import settings
from . import exceptions


class Resource(object):
    u"""リソース管理を受けるオブジェクトのベースクラス"""
    def __init__(self):
        if not Robot.connecting():
            raise exceptions.RobotConnectionError("No robot connection")


class ItemTypes(enum.Enum):
    u""""""
    JOINT_GROUP      = 'joint_group'
    MOBILE_BASE      = 'mobile_base'
    END_EFFECTOR     = 'end_effector'
    CAMERA           = 'camera'
    FORCE_TORQUE     = 'force_torque'
    IMU              = 'imu'
    LIDAR            = 'lidar'
    BATTERY          = 'power_supply'
    OBJECT_DETECTION = 'object_detection'
    COLLISION_WORLD  = 'collision_world'
    TEXT_TO_SPEECH   = 'text_to_speech'


class _ConnectionManager(object):
    u"""ロボットとの接続制御を行うオブジェクト"""

    def __init__(self):
        rospy.init_node('hsrb_interface_py', anonymous=True)
        self._tf2_buffer = tf2_ros.Buffer()
        self._tf2_listener = tf2_ros.TransformListener(self._tf2_buffer)
        self._registry = {}

    def __del__(self):
        self._tf2_listener = None
        self._tf2_buffer = None
        rospy.signal_shutdown('shutdown')

    @property
    def tf2_buffer(self):
        return weakref.proxy(self._tf2_buffer)

    def list(self, typ=None):
        u"""利用可能なアイテムを列挙する"""
        if typ is None:
            targets = [x for x in ItemTypes]
        else:
            targets = [typ]
        results = []
        for target in targets:
            section = settings.get_section(target.value)
            if section is None:
                raise exceptions.ResourceNotFoundError("No such category ({0})".format(target))
            for key in section.keys():
                results.append((key, target))
        return results

    def get(self, name, typ=None):
        u"""利用可能なアイテムのハンドルを生成する

        Attributes:
            name (str): リソース名
            typ (ItemTypes): アイテムカテゴリ
        """
        if typ is None:
            section, config = settings.get_entry_by_name(name)
            types = filter(lambda e: e.value == section, ItemTypes)
            if types:
                typ = types[0]
            else:
                raise exceptions.ResourceNotFoundError("No such category ({0})".format(section))
        key = (name, typ)
        if key in self._registry:
            return self._registry.get(key, None)
        else:
            config = settings.get_entry(typ.value, name)
            module_name, class_name = config["class"]
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

	from hsrb_interface import Robot, ItemTypes
        with Robot() as robot:
            print robot.list(ItemTypes.JOINT_GROUP)
            whole_body = robot.get("whole_body", ItemTypes.JOINT_GROUP)

    Attributes:
        name (str): ロボット名
    """
    _connection = None

    Items = ItemTypes

    @classmethod
    def connecting(cls):
        return cls._connection is not None and cls._connection() is not None

    @classmethod
    def _get_tf2_buffer(cls):
        if cls._connection is not None:
            conn = cls._connection()
            if conn is not None:
                return conn.tf2_buffer
            else:
                return None
        else:
            return None

    def __init__(self, *args, **kwargs):
        if Robot._connection is None or Robot._connection() is None:
            self._conn = _ConnectionManager()
            Robot._connection = weakref.ref(self._conn)
        else:
            self._conn = Robot._connection()

    def close(self):
        u"""直ちに接続を閉じる"""
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
        u"""ロボット名を返す"""
        return settings.get_entry('robot', 'hsrb')['fullname']

    def list(self, typ=None):
        u"""利用可能なアイテムをリストアップする。

        Returns:
            List[str]: 利用可能なリスト
        """
        return self._conn.list(typ)


    def get(self, name, typ=None):
        u"""利用可能なアイテムを取得する。
        Args:
            name (str): 取得したいオブジェクトの名前
            typ (Types): 取得したいオブジェクトの種類

        Returns:
            Resource: アイテムのインスタンス

        Raises:
            ResourceNotFoundError
        """
        return self._conn.get(name, typ)


def _get_tf2_buffer():
    return Robot._get_tf2_buffer()
