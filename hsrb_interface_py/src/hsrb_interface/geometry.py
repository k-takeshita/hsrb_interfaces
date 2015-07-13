#!/usr/bin/env python
# vim: fileencoding=utf-8
#
# Copyright (c) 2015, TOYOTA MOTOR CORPORATION
# All rights reserved.

import math
import collections

Vector3 = collections.namedtuple('Vector3', 'x y z')
Quaternion = collections.namedtuple('Quaternion', 'x y z w')

def from_ros_vector3(msg):
    return Vecotr3(msg.x, msg.y, msg.z)

def from_ros_quaternion(msg):
    return Quaternion(msg.x, msg.y, msg.z, msg.w)

def normalize_angle_positive(angle):
    twopi = 2.0 * math.pi
    return math.fmod(math.fmod(angle, twopi) + twopi, twopi)


def normalize_angle(angle):
    a = normalize_angle_positive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def shortest_angular_distance(_from, to):
    return normalize_angle(to - _from)

def tuples_to_pose(tuples):
    trans, rot = tuples
    pose = Pose()
    pose.position.x = trans[0]
    pose.position.y = trans[1]
    pose.position.z = trans[2]
    pose.orientation.x = rot[0]
    pose.orientation.y = rot[1]
    pose.orientation.z = rot[2]
    pose.orientation.w = rot[3]
    return pose

def pose_to_tuples(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    return (Vector3(x, y, z), Quaternion(qx, qy, qz, qw))

def tuples_to_transform(tuples):
    trans, rot = tuples
    transform = Transform()
    transform.translation.x = trans[0]
    transform.translation.y = trans[1]
    transform.translation.z = trans[2]
    transform.rotation.x = rot[0]
    transform.rotation.y = rot[1]
    transform.rotation.z = rot[2]
    transform.rotation.w = rot[3]
    return transform

def transform_to_tuples(transform):
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    return (Vector3(x, y, z), Quaternion(qx, qy, qz, qw))

def multiply_tuples(t1, t2):
    trans1, rot1 = t1
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = numpy.dot(trans1_mat, rot1_mat)

    trans2, rot2 = t2
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = numpy.dot(trans2_mat, rot2_mat)

    mat3 = numpy.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return (Vector3(*trans3), Quarternion(*rot3))
