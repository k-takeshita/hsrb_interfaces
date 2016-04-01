#!/usr/bin/env python
# vim: fileencoding=utf-8

import os
import rospy
from tmc_manipulation_msgs.srv import (
    GetCollisionEnvironment,
    GetCollisionEnvironmentRequest,
)
from tmc_manipulation_msgs.msg import (
    CollisionObject,
    CollisionObjectOperation,
    CollisionEnvironment,
)

from tmc_msgs.msg import (
    ObjectIdentifierArray,
)

from tmc_geometric_shapes_msgs.msg import Shape
from hsrb_interface import geometry

from . import robot
from . import utils
from . import settings

# トピック受信待ちタイムアウト[sec]
_WAIT_TOPIC_TIMEOUT = 20.0

class CollisionWorld(robot.Item):
    u"""衝突検知用環境インターフェース

    Attributes:
        known_object_only (bool):
        ref_frame_id (str):


    """
    def __init__(self, name):
        super(CollisionWorld, self).__init__()
        self._setting = settings.get_entry('collision_world', name)
        self._known_object_only = True
        self._ref_frame_id = settings.get_frame('map')
        self._collision_object_pub = rospy.Publisher(self._setting['topic'],
                                                     CollisionObject,
                                                     queue_size=100)
        self._environment = CollisionEnvironment()
        self._object_pub = rospy.Publisher('known_object', CollisionObject, queue_size=100)
        # デフォルトではobject_idが10000から始まる
        self._start_object_id = 10000
        self._object_count = self._start_object_id
        self._known_object_ids_sub = utils.CachingSubscriber('known_object_ids', ObjectIdentifierArray, default=ObjectIdentifierArray())
        self._known_object_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)

    def _is_object_id_used(self, id):
        return id in [x.object_id for x in self._known_object_ids_sub.data.object_ids]

    def _wait_object_id_used(self, id, timeout=1.0):
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            if self._is_object_id_used(id):
                return True
            if rospy.Time.now() - start > rospy.Duration(timeout):
                return False
            rospy.sleep(0.1)

    @property
    def known_object_only(self):
        return self._known_object_only

    @known_object_only.setter
    def known_object_only(self, value):
        self._known_object_only = value

    @property
    def ref_frame_id(self):
        return self._ref_frame_id

    @ref_frame_id.setter
    def ref_frame_id(self, value):
        self._ref_frame_id = value

    @property
    def next_object_id(self):
        return self._object_count

    @next_object_id.setter
    def next_object_id(self, value):
        self._start_object_id = value
        self._object_count = self._start_object_id

    @property
    def environment(self):
        u"""tmc_manipulation_msgs.msg.CollisionEnvironment: 最後に取得した衝突検知用の空間"""
        return self._environment

    def snapshot(self, ref_frame_id=None):
        u"""現在の衝突検知用の空間を取得する

        Args:
            ref_frame_id (str):　基準となるframe。ref_frame_id属性よりも引数が優先される。

        Returns:
            tmc_manipulation_msgs.msg.CollisionEnvironment: 衝突検知用の空間
        """
        req = GetCollisionEnvironmentRequest()
        req.known_object_only = self._known_object_only
        if ref_frame_id is None:
            req.origin_frame_id = self._ref_frame_id
        else:
            req.origin_frame_id = ref_frame_id

        service = rospy.ServiceProxy(self._setting['service'],
                                     GetCollisionEnvironment)
        res = service.call(req)
        self._environment = res.environment
        return self._environment

    def add_box(self, x=0.1, y=0.1, z=0.1, pose=geometry.create_pose(), frame_id='map', name='box'):
        u"""干渉物体の箱を追加する

        Args:
            x: 縦[m]
            y: 横[m]
            z: 高さ[m]
            pose: frame_id基準の位置姿勢
            frame_id: 物体が属するframe
        Returns:
            Tuple[int, str]: 追加物体のID
        """
        box = CollisionObject()
        shape = Shape()
        shape.type = Shape.BOX
        shape.dimensions = [x, y, z]
        pose = geometry.tuples_to_pose(pose)
        box.operation.operation = CollisionObjectOperation.ADD
        self._known_object_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count + 1
        box.id.object_id = self._object_count
        box.id.name = name
        box.shapes = [shape]
        box.poses = [pose]
        box.header.frame_id = frame_id
        box.header.stamp = rospy.Time.now()
        self._object_pub.publish(box)
        # 反映されるまで待ち
        if self._wait_object_id_used(self._object_count):
            return (box.id.object_id, box.id.name)
        else:
            return ()

    def add_sphere(self, radius=0.1, pose=geometry.create_pose(), frame_id='map', name='sphere'):
        u"""干渉物体の球を追加する

        Args:
            radius: 半径[m]
            pose: frame_id基準の位置姿勢
            frame_id: 物体が属するframe
        Returns:
            Tuple[int, str]: 追加物体のID
        """
        sphere = CollisionObject()
        shape = Shape()
        shape.type = Shape.SPHERE
        shape.dimensions = [radius]
        pose = geometry.tuples_to_pose(pose)
        sphere.operation.operation = CollisionObjectOperation.ADD
        self._known_object_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count + 1
        sphere.id.object_id = self._object_count
        sphere.id.name = name
        sphere.shapes = [shape]
        sphere.poses = [pose]
        sphere.header.frame_id = frame_id
        sphere.header.stamp = rospy.Time.now()
        self._object_pub.publish(sphere)
        # 反映されるまで待ち
        if self._wait_object_id_used(self._object_count):
            return (sphere.id.object_id, sphere.id.name)
        else:
            return ()

    def add_cylinder(self, radius=0.1, length=0.1, pose=geometry.create_pose(), frame_id='map', name='cylinder'):
        u"""干渉物体の円柱を追加する

        Args:
            radius: 半径[m]
            length: 高さ[m]
            pose: frame_id基準の位置姿勢
            frame_id: 物体が属するframe
        Returns:
            Tuple[int, str]: 追加物体のID
        """
        cylinder = CollisionObject()
        shape = Shape()
        shape.type = Shape.CYLINDER
        shape.dimensions = [radius, length]
        pose = geometry.tuples_to_pose(pose)
        cylinder.operation.operation = CollisionObjectOperation.ADD
        self._known_object_ids_sub.wait_for_message(_WAIT_TOPIC_TIMEOUT)
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count + 1
        cylinder.id.object_id = self._object_count
        cylinder.id.name = name
        cylinder.shapes = [shape]
        cylinder.poses = [pose]
        cylinder.header.frame_id = frame_id
        cylinder.header.stamp = rospy.Time.now()
        self._object_pub.publish(cylinder)
        # 反映されるまで待ち
        if self._wait_object_id_used(self._object_count):
            return (cylinder.id.object_id, cylinder.id.name)
        else:
            return ()

    def add_mesh(self, filename, pose=geometry.create_pose(), frame_id='map', name='mesh'):
        u"""干渉物体のメッシュを追加する

        Args:
            filename: stlのファイル名。
                      例：'http://hoge/mesh.stl, package://your_pkg/mesh/hoge.stl, file:///home/hoge/huge.stl'
            pose: frame_id基準の位置姿勢
            frame_id: 物体が属するframe
        Returns:
            Tuple[int, str]: 追加物体のID
        Raises:
            IOError: ファイルが存在しない
        """
        mesh = CollisionObject()
        shape = Shape()
        shape.type = Shape.MESH
        shape.stl_file_name = filename
        pose = geometry.tuples_to_pose(pose)
        mesh.operation.operation = CollisionObjectOperation.ADD
        while self._is_object_id_used(self._object_count):
            self._object_count = self._object_count + 1
        mesh.id.object_id = self._object_count
        mesh.id.name = name
        mesh.shapes = [shape]
        mesh.poses = [pose]
        mesh.header.frame_id = frame_id
        mesh.header.stamp = rospy.Time.now()
        self._object_pub.publish(mesh)
        # 反映されるまで待ち
        if self._wait_object_id_used(self._object_count):
            return (mesh.id.object_id, mesh.id.name)
        else:
            return ()

    def remove(self, object_id):
        u"""衝突検知用の空間から物体を削除

        Args:
            object_id (Tuple[int, str]): 物体ID
        Returns:
            None
        """
        number, name = object_id
        collision_object = CollisionObject()
        collision_object.id.object_id = number
        collision_object.id.name = name
        collision_object.operation.operation = CollisionObjectOperation.REMOVE
        self._collision_object_pub.publish(collision_object)

    def remove_all(self):
        u"""干渉物体をクリアする"""
        clear = CollisionObject()
        clear.operation.operation = CollisionObjectOperation.REMOVE
        clear.id.object_id = 0
        self._object_count = self._start_object_id
        clear.id.name = 'all'
        clear.header.stamp = rospy.Time.now()
        self._object_pub.publish(clear)
