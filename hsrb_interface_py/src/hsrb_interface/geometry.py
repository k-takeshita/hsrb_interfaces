# vim: fileencoding=utf-8
"""Simple Geometry Library.

Copyright (c) 2015, TOYOTA MOTOR CORPORATION All rights reserved.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import collections
import math

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform
import numpy as np
import tf


Vector3 = collections.namedtuple('Vector3', 'x y z')
Quaternion = collections.namedtuple('Quaternion', 'x y z w')


def pose(x=0.0, y=0.0, z=0.0, ei=0.0, ej=0.0, ek=0.0, axes='sxyz'):
    """Create pose tuple.

    Args:
        x, y, z: Linear translation.
        ei, ej, ek, axes: Rotation in euler form.
            By default, (ei, ej, ek) are correspond to (roll, pitch, yaw).

    Returns:
        Tuple[Vector3, Quaternion]: A new pose.
    """
    vec3 = (x, y, z)
    quaternion = tf.transformations.quaternion_from_euler(ei, ej, ek, axes)
    return (Vector3(*vec3), Quaternion(*quaternion))

# For backward compatibility
create_pose = pose


def vector3(x=0.0, y=0.0, z=0.0):
    """Construct a Vector3 instance."""
    return Vector3(x, y, z)


def quaterion(x=0.0, y=0.0, z=0.0, w=1.0):
    """Construct a Quaternion instance."""
    return Quaternion(x, y, z, w)


def from_ros_vector3(msg):
    """Convert ``geometry_msgs/Vector3`` to a :py:class:`Vector3`.

    Args:
        msg (geometry_msgs.msg.Vector3): A ROS message.
    """
    return Vector3(msg.x, msg.y, msg.z)


def from_ros_quaternion(msg):
    """
    """
    return Quaternion(msg.x, msg.y, msg.z, msg.w)


def normalize_angle_positive(angle):
    """
    """
    twopi = 2.0 * math.pi
    return math.fmod(math.fmod(angle, twopi) + twopi, twopi)


def normalize_angle(angle):
    """
    """
    a = normalize_angle_positive(angle)
    if a > math.pi:
        a -= 2.0 * math.pi
    return a


def shortest_angular_distance(_from, to):
    """
    """
    return normalize_angle(to - _from)


def tuples_to_pose(tuples):
    """
    """
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
    """
    """
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w
    return (Vector3(x, y, z), Quaternion(qx, qy, qz, qw))


def tuples_to_transform(tuples):
    """
    """
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
    """
    """
    x = transform.translation.x
    y = transform.translation.y
    z = transform.translation.z
    qx = transform.rotation.x
    qy = transform.rotation.y
    qz = transform.rotation.z
    qw = transform.rotation.w
    return (Vector3(x, y, z), Quaternion(qx, qy, qz, qw))


def multiply_tuples(t1, t2):
    """
    """
    trans1, rot1 = t1
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2, rot2 = t2
    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)

    return (Vector3(*trans3), Quaternion(*rot3))
