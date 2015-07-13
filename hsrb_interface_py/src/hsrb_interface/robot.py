#!/usr/bin/env python
# vim: fileencoding=utf-8
#
# Copyright (c) 2015, TOYOTA MOTOR CORPORATION
# All rights reserved.
#
import rospy


from .utils import CachingSubscriber
from .settings import settings
from .exceptions import RobotConnectionError

import weakref


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
        u"""ロボット名を
        """
        return settings['robot']['name']

    def list_joint_group(self):
        u"""利用可能なジョイントグループをリストアップする。

        Returns:
            List[str]: 利用可能なジョイントグループ名のリスト
        """
        return [r.name for r in settings['joint_group']]

    def list_mobile_base(self):
        u"""利用可能な移動台座をリストアップする。

        Returns:
            List[str]: 利用可能な移動台座名のリスト
        """
        return [r.name for r in settings['mobile_base']]

    def list_camera(self):
        u"""利用可能なカメラをリストアップする。

        Returns:
            List[str]: 利用可能なカメラ名のリスト
        """
        return [r.name for r in settings['camera']]

    def list_laser_scan(self):
        u"""利用可能なレーザースキャナーをリストアップする。

        Returns:
            List[str]: 利用可能なレーザースキャナー名のリスト
        """
        return [r.name for r in settings['laser_scan']]

    def list_imu(self):
        u"""利用可能な慣性センサーをリストアップする。

        Returns:
            List[str]: 利用可能な慣性センサー名のリスト
        """
        return [r.name for r in settings['imu']]

    def list_force_torque(self):
        u"""利用可能な6軸力センサーをリストアップする。

        Returns:
            List[str]: 利用可能な6軸力センサー名のリスト
        """
        return [r.name for r in settings['force_torque']]

    def list_power_source(self):
        u"""利用可能な電源をリストアップする。

        Returns:
            List[str]: 利用可能な電源名のリスト
        """
        return [r.name for r in settings['power_source']]

    def list_end_effector(self):
        u"""利用可能なエンドエフェクターをリストアップする。

        Returns:
            List[str]: 利用可能なエンドエフェクターのリスト
        """
        return [r.name for r in settings['end_effector']]

    def list_object_detector(self):
        u"""利用可能な物体認識器をリストアップする。

        Returns:
            List[str]: 利用可能な物体認識器のリスト
        """
        return [r.name for r in settings['object_detector']]

    def list_collision_map(self):
        u"""利用可能な物体干渉マップをリストアップする。

        Returns:
            List[str]: 利用可能な物体干渉マップのリスト
        """
        return [r.name for r in settings['collision_map']]

    def list_text_to_speech(self):
        u"""利用可能な音声合成サービスをリストアップする。

        Returns:
            List[str]: 利用可能な音声合成サービスのリスト
        """
        return [r.name for r in settings['text_to_speech']]


class Resource(object):
    u"""リソース管理を受けるオブジェクトのベースクラス

    """
    def __init__(self):
        if Robot._connection is None or Robot._connection() is None:
            raise RobotConnectionError("Noe robot connection")
