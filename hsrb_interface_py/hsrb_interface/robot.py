# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""This module provides classes and functions to manage connections to robots.

Copyright (c) 2015-2016 Toyota Motor Corporation
"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import enum
import importlib
import sys
import warnings
import weakref

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import tf2_ros

from . import exceptions
from . import settings


class Item(rclpy.node.Node):
    """A base class to be under resource management.

    Raises:
        hsrb_interface.exceptions.RobotConnectionError:
            Not connected to a robot.
    """

    def __init__(self):
        self._node = Robot._connection
        """See class docstring."""
        if not Robot._connecting():
            raise exceptions.RobotConnectionError("No robot connection")


class ItemTypes(enum.Enum):
    """Types of resource items.

    Attributes:
        JOINT_GROUP:
        END_EFFECTOR:
        MOBILE_BASE:
        CAMERA:
        FORCE_TORQUE:
        IMU:
        LIDAR:
        BATTERY:
        OBJECT_DETECTION:
        COLLISION_WORLD:
        TEXT_TO_SPEECH:

    Warnings:
        This class is deprecated.
    """

    JOINT_GROUP = 'joint_group'
    MOBILE_BASE = 'mobile_base'
    END_EFFECTOR = 'end_effector'
    CAMERA = 'camera'
    FORCE_TORQUE = 'force_torque'
    IMU = 'imu'
    LIDAR = 'lidar'
    BATTERY = 'power_supply'
    OBJECT_DETECTION = 'object_detection'
    COLLISION_WORLD = 'collision_world'
    TEXT_TO_SPEECH = 'text_to_speech'


def _type_deprecation_warning(name, typ):
    """Warn uses of ItemType feature."""
    if typ is not None:
        msg = "A feature specifying an item by ItemType is deprecated."
        warnings.warn(msg, exceptions.DeprecationWarning)
    if name == "default":
        if typ is ItemTypes.TEXT_TO_SPEECH:
            return "default_tts"
        elif typ is ItemTypes.COLLISION_WORLD:
            return "global_collision_world"
    return name


_interactive = False


def _is_interactive():
    """True if interactive mode is set.

    Returns:
        bool: Is interactive mode enabled or not.
    """
    return _interactive


def enable_interactive():
    """Enable interactive mode.

    This function changes behavior when a user send SIGINT(Ctrl-C).
    In the interactive mode, SIGINT doesn't stop process but cancel
    executing action or other blocking procedure.

    Returns:
        None
    """
    global _interactive
    _interactive = True


class _ConnectionManager(Node):
    """This class manage connection with a robot.

    Basically, only 1 instance should be created at 1 process.
    Usually ``Robot`` instances manage this object.
    All of ``Resource`` subclass instances are owned by this class and
    synchronize their lifecycle.

    Args:
        use_tf_client: Use action based query for tf.(bool)

    """

    def __init__(self, use_tf_client=False):
        """See class docstring."""
        context = rclpy.utilities.get_default_context()
        if not context.ok():
            rclpy.init()
        super().__init__('hsrb_interface_py')
        if use_tf_client:
            self._tf2_buffer = tf2_ros.BufferClient('/tf2_buffer_server')
        else:
            qos_profile = QoSProfile(depth=1)
            self._tf2_buffer = tf2_ros.Buffer()
            self._tf2_listener = tf2_ros.TransformListener(
                self._tf2_buffer, self, qos=qos_profile)
        self._registry = {}

    def __del__(self):
        self._tf2_listener = None
        self._tf2_buffer = None

    @property
    def tf2_buffer(self):
        return weakref.proxy(self._tf2_buffer)

    def list(self, typ=None):
        """List available items.

        Args:
            typ (ItemTypes):
        """
        if typ is None:
            targets = [x for x in ItemTypes]
        else:
            targets = [typ]
        results = []
        for target in targets:
            section = settings.get_section(target.value)
            if section is None:
                msg = "No such category ({0})".format(target)
                raise exceptions.ResourceNotFoundError(msg)
            for key in section.keys():
                results.append((key, target))
        return results

    def get(self, name, typ=None):
        """Get an item if available.

        Args:
            name (str):   A name of ``Item`` to get.
            typ (Types):  A type of ``Item`` to get.

        Returns:
            Item: An instance with a specified name

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError
        """
        if typ is None:
            section, config = settings.get_entry_by_name(name)
            types = list(filter(lambda e: e.value == section, ItemTypes))
            if types:
                typ = types[0]
            else:
                msg = "No such category ({0})".format(section)
                raise exceptions.ResourceNotFoundError(msg)
        key = (name, typ)
        if key in self._registry:
            return self._registry.get(key, None)
        else:
            config = settings.get_entry(typ.value, name)
            module_name, class_name = config["class"]
            module = importlib.import_module(".{0}".format(module_name),
                                             "hsrb_interface")
            cls = getattr(module, class_name)
            obj = cls(name)
            self._registry[key] = obj
            return weakref.proxy(obj)


def _get_tf2_buffer():
    """Get global tf2 buffer."""
    return Robot._get_tf2_buffer()


class Robot(object):
    """A handle object to manage a robot connection.

    Args:
        *args: Reserved for future use.
        **kwargs: use_tf_client[bool] Use action based query for tf.

    This class allow multiple instances. In that case, the connection is closed
    if all instances are destroyed or invoke :py:meth:`.close()` method.

    In order to establish a new connection, you need to create a new instance.

    Attributes:
        name (str): A name of a connecting robot.

    Example:
        .. sourcecode:: python

           from hsrb_interface import Robot, ItemTypes
                with Robot() as robot:
                    print(robot.list())
                    whole_body = robot.get("whole_body")
    """

    _connection = None

    @property
    def Items(self):
        """Repesent types of resource items.

        Warnings:
            This class is deprecated. It will be removed in future release.
        """
        msg = "A feature specifying a resource item by ItemType is deprecated."
        warnings.warn(msg, exceptions.DeprecationWarning)
        return ItemTypes

    @classmethod
    def _connecting(cls):
        return cls._connection is not None

    @classmethod
    def connecting(cls):
        """Check whether the connection to a robot is valid."""
        warnings.warn("Robot.connectiong() is depreacated",
                      exceptions.DeprecationWarning)
        return cls._connecting()

    @classmethod
    def _get_tf2_buffer(cls):
        if cls._connection is not None:
            conn = cls._connection
            if conn is not None:
                return conn.tf2_buffer
            else:
                return None
        else:
            return None

    def __init__(self, *args, **kwargs):
        """See class docstring."""
        use_tf_client = kwargs.get('use_tf_client', False)
        if Robot._connection is None:
            self._conn = _ConnectionManager(use_tf_client=use_tf_client)
            Robot._connection = self._conn
        else:
            self._conn = Robot._connection

    def close(self):
        """Shutdown immediately."""
        self.__exit__(None, None, None)

    def __enter__(self):
        """A part of ContextManager interface."""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """A part of ContextManager interface."""
        self._conn.destroy_node()
        self._conn = None
        Robot._connection = None

    def ok(self):
        """Check whether this handle is valid or not.

        Returns:
            bool: ``True`` if this handle is valid. Otherwise False.
        """
        return self._conn is not None

    def _get_name(self):
        return settings.get_entry('robot', 'hsrb')['fullname']
    name = property(_get_name)

    def list(self, typ=None):
        """List available items up.

        Args:
            typ (Types):  A type of ``Item`` to list.
                If None, all types are selected.

        Returns:
            List[str]: A list of available items.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        if typ is not None:
            msg = """A feature specifying an item by ItemType is deprecated."""
            warnings.warn(msg, exceptions.DeprecationWarning)
        return self._conn.list(typ)

    def get(self, name, typ=None):
        """Get an item if available.

        Args:
            name (str):   A name of an item to get.
            typ (Types):  A type of an item to get.

        Returns:
            Item: An instance with a specified name.

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError:
                A resource that named as `name` is not found.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        name = _type_deprecation_warning(name, typ)
        return self._conn.get(name)

    def try_get(self, name, typ=None, msg="Ignored"):
        """Try to get an item if available.

        If trial failed, error messsage is printed to ``stderr`` instead of
        raising exception.

        Args:
            name (str):   A name of ``Item`` to get.
            typ (Types):  A type of ``Item`` to get.
            msg (str):  A error message. (If msg is None, output nothing）

        Returns:
            Item: An instance with a specified name.

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError:
                A resource that named as `name` is not found.

        Warnings:
            `typ` parameter is deprecated.
            It will be removed in future release.
        """
        name = _type_deprecation_warning(name, typ)
        try:
            return self.get(name, None)
        except (exceptions.ResourceNotFoundError,
                exceptions.RobotConnectionError):
            if msg is not None:
                err = "Failed to get Item({0}): {1}".format(name, msg)
                print(err, file=sys.stderr)
