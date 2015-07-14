#!/usr/bin/env python
# vim: fileencoding=utf-8
#
# Copyright (c) 2015, TOYOTA MOTOR CORPORATION
# All rights reserved.
#
from __future__ import absolute_import
import weakref
import rospy


from . import utils
from . import settings
from . import exceptions


class _Connection(object):
    u"""ロボットとの接続制御を行うオブジェクト"""
    def __init__(self):
        rospy.init_node('hsrb_interface_py', anonymous=True)

    def __del__(self):
        rospy.signal_shutdown('shutdown')


class Robot(object):
    u"""ロボットとの接続を管理するハンドル

    複数のインスタンスを作ることができ、その場合最後のインスタンスで`close`が
    呼び出されるか、すべてのインスタンスが破壊されると接続が切断される。

    新たに接続を確立するには、新しいインスタンスを生成する必要がある。

    Example:

        with Robot() as r:
            print r.list_camera()

    Attributes:
        name (str): ロボット名
    """

    _connection = None

    def __init__(self, *args, **kwargs):
        if Robot._connection is None or Robot._connection() is None:
            self._conn = _Connection()
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
        u"""ロボット名を返す
        """
        return settings.get_entry('robot', 'hsrb')['fullname']

    def list_joint_group(self):
        u"""利用可能なジョイントグループをリストアップする。

        Returns:
            List[str]: 利用可能なジョイントグループ名のリスト
        """
        return settings.get_section('joint_group').keys()

    def list_mobile_base(self):
        u"""利用可能な移動台座をリストアップする。

        Returns:
            List[str]: 利用可能な移動台座名のリスト
        """
        return settings.get_section('mobile_base').keys()

    def list_camera(self):
        u"""利用可能なカメラをリストアップする。

        Returns:
            List[str]: 利用可能なカメラ名のリスト
        """
        return settings.get_section('camera').keys()

    def list_lidar(self):
        u"""利用可能なレーザースキャナーをリストアップする。

        Returns:
            List[str]: 利用可能なレーザースキャナー名のリスト
        """
        return settings.get_section('lidar').keys()

    def list_imu(self):
        u"""利用可能な慣性センサーをリストアップする。

        Returns:
            List[str]: 利用可能な慣性センサー名のリスト
        """
        return settings.get_section('imu').keys()

    def list_force_torque(self):
        u"""利用可能な6軸力センサーをリストアップする。

        Returns:
            List[str]: 利用可能な6軸力センサー名のリスト
        """
        return settings.get_section('force_torque').keys()

    def list_power_supply(self):
        u"""利用可能な電源をリストアップする。

        Returns:
            List[str]: 利用可能な電源名のリスト
        """
        return settings.get_section('power_supply').keys()

    def list_end_effector(self):
        u"""利用可能なエンドエフェクターをリストアップする。

        Returns:
            List[str]: 利用可能なエンドエフェクターのリスト
        """
        return settings.get_section('end_effector').keys()

    def list_object_detector(self):
        u"""利用可能な物体認識器をリストアップする。

        Returns:
            List[str]: 利用可能な物体認識器のリスト
        """
        return settings.get_section('object_detector').keys()

    def list_collision_map(self):
        u"""利用可能な物体干渉マップをリストアップする。

        Returns:
            List[str]: 利用可能な物体干渉マップのリスト
        """
        return settings.get_section('collision_map').keys()

    def list_text_to_speech(self):
        u"""利用可能な音声合成サービスをリストアップする。

        Returns:
            List[str]: 利用可能な音声合成サービスのリスト
        """
        return settings.get_section('text_to_speech').keys()


class Resource(object):
    u"""リソース管理を受けるオブジェクトのベースクラス

    """
    def __init__(self):
        if Robot._connection is None or Robot._connection() is None:
            raise exceptions.RobotConnectionError("No robot connection")
