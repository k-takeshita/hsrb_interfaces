# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for collision_world module"""
from __future__ import absolute_import

import os
import sys

import hsrb_interface

from hsrb_interface import geometry

import hsrb_interface.collision_world
import hsrb_interface.exceptions

from mock import ANY
from mock import MagicMock
from mock import PropertyMock
from nose.tools import eq_

import testing

from tmc_geometric_shapes_msgs.msg import Shape
from tmc_manipulation_msgs.msg import CollisionObject
from tmc_manipulation_msgs.msg import CollisionObjectOperation
from tmc_manipulation_msgs.srv import GetCollisionEnvironment
from tmc_manipulation_msgs.srv import GetCollisionEnvironmentRequest
from tmc_msgs.msg import ObjectIdentifier
from tmc_msgs.msg import ObjectIdentifierArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class CollisionWorldTest(testing.RosMockTestCase):

    def test_creation(self):
        """Test simple use case of MobileBase class"""
        self.create()
        self.publisher_mock.assert_called_with("/known_object",
                                               CollisionObject,
                                               queue_size=ANY)
        self.subscriber_mock.assert_called_with("/known_object_ids",
                                                ObjectIdentifierArray,
                                                callback=ANY)

    def create(self):
        """Create a CollisionWorld instance"""
        self.get_frame_mock.return_value = 'map'
        self.get_entry_mock.return_value = {
            "class": ["collision_world", "CollisionWorld"],
            "service": "/get_collision_environment",
            "control_topic": "/known_object",
            "listing_topic": "/known_object_ids",
        }
        return hsrb_interface.collision_world.CollisionWorld('test')

    def test_snapshot(self):
        collision_world = self.create()
        proxy_instance_mock = self.service_proxy_mock.return_value
        collision_world.snapshot()
        self.service_proxy_mock.assert_called_with(
            "/get_collision_environment",
            GetCollisionEnvironment)
        req = GetCollisionEnvironmentRequest()
        req.known_object_only = True
        req.origin_frame_id = 'map'
        proxy_instance_mock.call.assert_called_with(req)

    def test_add_box(self):
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        next_id = collision_world.next_object_id
        known_ids = ObjectIdentifierArray()
        known_id = ObjectIdentifier(object_id=next_id, name='box')
        known_ids.object_ids.append(known_id)
        data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
                                              known_ids])
        # PropertyMock must be attached to class
        type(sub_mock).data = data_mock

        pose = geometry.pose(1, 2, 3)
        id, name = collision_world.add_box(pose=pose)
        eq_(id, next_id)
        eq_(name, 'box')
        shape = Shape()
        shape.type = Shape.BOX
        shape.dimensions = [0.1, 0.1, 0.1]
        msg = CollisionObject()
        msg.id = ObjectIdentifier(object_id=next_id, name='box')
        msg.header.stamp = ANY
        msg.header.frame_id = 'map'
        msg.operation.operation = CollisionObjectOperation.ADD
        msg.shapes.append(shape)
        msg.poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_sphere(self):
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        next_id = collision_world.next_object_id
        known_ids = ObjectIdentifierArray()
        known_id = ObjectIdentifier(object_id=next_id, name='sphere')
        known_ids.object_ids.append(known_id)
        data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
                                              known_ids])
        # PropertyMock must be attached to class
        type(sub_mock).data = data_mock

        pose = geometry.pose(1, 2, 3)
        id, name = collision_world.add_sphere(pose=pose)
        eq_(id, next_id)
        eq_(name, 'sphere')
        shape = Shape()
        shape.type = Shape.SPHERE
        shape.dimensions = [0.1]
        msg = CollisionObject()
        msg.id = ObjectIdentifier(object_id=next_id, name='sphere')
        msg.header.stamp = ANY
        msg.header.frame_id = 'map'
        msg.operation.operation = CollisionObjectOperation.ADD
        msg.shapes.append(shape)
        msg.poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_cylinder(self):
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        next_id = collision_world.next_object_id
        known_ids = ObjectIdentifierArray()
        known_id = ObjectIdentifier(object_id=next_id, name='cylinder')
        known_ids.object_ids.append(known_id)
        data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
                                              known_ids])
        # PropertyMock must be attached to class
        type(sub_mock).data = data_mock

        pose = geometry.pose(1, 2, 3)
        id, name = collision_world.add_cylinder(pose=pose)
        eq_(id, next_id)
        eq_(name, 'cylinder')
        shape = Shape()
        shape.type = Shape.CYLINDER
        shape.dimensions = [0.1, 0.1]
        msg = CollisionObject()
        msg.id = ObjectIdentifier(object_id=next_id, name='cylinder')
        msg.header.stamp = ANY
        msg.header.frame_id = 'map'
        msg.operation.operation = CollisionObjectOperation.ADD
        msg.shapes.append(shape)
        msg.poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_add_mesh_success(self):
        pub_mock = self.publisher_mock.return_value
        sub_mock = MagicMock()
        collision_world = self.create()
        collision_world._known_obj_ids_sub = sub_mock

        next_id = collision_world.next_object_id
        known_ids = ObjectIdentifierArray()
        known_id = ObjectIdentifier(object_id=next_id, name='mesh')
        known_ids.object_ids.append(known_id)
        data_mock = PropertyMock(side_effect=[ObjectIdentifierArray(),
                                              known_ids])
        # PropertyMock must be attached to class
        type(sub_mock).data = data_mock

        pose = geometry.pose(1, 2, 3)
        id, name = collision_world.add_mesh('hoge.stl', pose=pose)
        eq_(id, next_id)
        eq_(name, 'mesh')
        shape = Shape()
        shape.type = Shape.MESH
        shape.stl_file_name = 'hoge.stl'
        msg = CollisionObject()
        msg.id = ObjectIdentifier(object_id=next_id, name='mesh')
        msg.header.stamp = ANY
        msg.header.frame_id = 'map'
        msg.operation.operation = CollisionObjectOperation.ADD
        msg.shapes.append(shape)
        msg.poses.append(geometry.tuples_to_pose(pose))
        pub_mock.publish.assert_called_with(msg)

    def test_remove(self):
        pub_mock = self.publisher_mock.return_value
        collision_world = self.create()
        collision_world.remove((42, 'answer'))
        msg = CollisionObject()
        msg.id.object_id = 42
        msg.id.name = 'answer'
        msg.operation.operation = CollisionObjectOperation.REMOVE
        pub_mock.publish.assert_called_with(msg)

    def test_remove_all(self):
        pub_mock = self.publisher_mock.return_value
        collision_world = self.create()
        collision_world.remove_all()
        msg = CollisionObject()
        msg.id.object_id = 0
        msg.id.name = ANY
        msg.header.stamp = ANY
        msg.operation.operation = CollisionObjectOperation.REMOVE
        pub_mock.publish.assert_called_with(msg)
