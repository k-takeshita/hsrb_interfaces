# vim: fileencoding=utf-8
"""
:copyright: (c) 2015 Toyota Motor Corporation
:license: TMC Proprietary License

"""
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import sys
import weakref
import rospy
import importlib
import enum
import tf2_ros

from . import settings
from . import exceptions


class Item(object):
    """A base class to be under resource management"""
    def __init__(self):
        if not Robot.connecting():
            raise exceptions.RobotConnectionError("No robot connection")


class ItemTypes(enum.Enum):
    """Types of resources

    Attributes:
        JOINT_GROUP:
        MOBILE_BASE:
        END_EFFECTOR:
        CAMERA:
        FORCE_TORQUE:
        IMU:
        LIDAR:
        BATTERY:
        OBJECT_DETECTION:
        COLLISION_WORLD:
        TEXT_TO_SPEECH:
    """
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


_interactive = False
def _is_interactive():
    """True if interactive mode is set.

    Returns:
        bool: Is interactive mode enabled or not
    """
    return _interactive

def enable_interactive():
    """Enable interactive mode

    Returns:
        None
    """
    global _interactive
    _interactive = True


class _ConnectionManager(object):
    """This class manage connection with a robot.

    Basically, only 1 instance should be created at 1 process.
    Usually ``Robot`` instances manage this object.
    All of ``Resource`` subclass instances are owned by this class and
    synchronize their lifecycle.

    """

    def __init__(self):
        try:
            master = rospy.get_master()
            uri = master.getUri("hsrb_interface_py")
        except Exception as e:
            raise exceptions.RobotConnectionError(e)
        disable_signals = _is_interactive()
        rospy.init_node('hsrb_interface_py', anonymous=True, disable_signals=disable_signals)
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
        """List available items"""
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


def _get_tf2_buffer():
    """Get global tf2 buffer."""
    return Robot._get_tf2_buffer()


class Robot(object):
    """A hand to manage a robot connection.

    This class allow multiple instances. In that case, the connection is closed
    if all instances are destroyed or invoke ``close`` method.

    In order to establish a new connection, you need to create a new instance.

    Example:

        .. sourcecode:: python

	        from hsrb_interface import Robot, ItemTypes
                with Robot() as robot:
                    print(robot.list(ItemTypes.JOINT_GROUP))
                    whole_body = robot.get("whole_body", ItemTypes.JOINT_GROUP)
    """
    _connection = None

    # For backward compatibility
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
        """Shutdown immediately."""
        self.__exit__(None, None, None)

    def __enter__(self):
        """A part of ContextManager interface"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """A part of ContextManager interface"""
        self._conn = None

    def ok(self):
        """``True`` if initialization succeeded."""
        return self._conn is not None

    @property
    def name(self):
        """A name of a connecting robot.

        Returns:
            str: A robot name
        """
        return settings.get_entry('robot', 'hsrb')['fullname']

    def list(self, typ=None):
        """List available items up.

        Returns:
            List[str]: A list of available items.
        """
        return self._conn.list(typ)


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
        return self._conn.get(name, typ)

    def try_get(self, name, typ=None, msg="Ignored"):
        """Try to get an item if available.

        If trial failed, error messsage is printed to ``stderr`` instead of
        raising exception.

        Args:
            name (str):   A name of ``Item`` to get.
            typ (Types):  A type of ``Item`` to get.
            msg (str):  A error message (If msg is None, output nothing）

        Returns:
            Item: An instance with a specified name

        Raises:
            hsrb_interface.exceptions.ResourceNotFoundError
        """
        try:
            return self.get(name, typ)
        except (exceptions.ResourceNotFoundError, exceptions.RobotConnectionError):
            if msg is not None:
                err = "Failed to get Item({0} : {1}): {2}".format(name, typ.value if typ else "Any", msg)
                print(err, file=sys.stderr)

