# Copyright (C) 2016 Toyota Motor Corporation
# vim: fileencoding=utf-8
"""Mapping table of ROS Graph names.

This module is intended to internal use only.
"""

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import json

from . import exceptions

VERSION = "1.0.0"


_HSRB_SETTINGS = """
{
    "robot": {
        "hsrb": {
            "fullname": "HSR-B"
        }
    },
    "frame": {
        "map": {
            "frame_id": "map"
        },
        "odom": {
            "frame_id": "odom"
        },
        "base": {
            "frame_id": "base_footprint"
        },
        "hand": {
            "frame_id": "hand_palm_link"
        }
    },
    "trajectory": {
            "impedance_control": "/hsrb/impedance_control",
            "constraint_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "whole_timeopt_filter_service": "/timeopt_filter_node/filter_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
    },
    "joint_group": {
        "whole_body": {
            "class":                        ["joint_group", "JointGroup"],
            "joint_states_topic":           "/joint_states",
            "arm_controller_prefix":        "/arm_trajectory_controller",
            "head_controller_prefix":       "/head_trajectory_controller",
            "hand_controller_prefix":       "/gripper_controller",
            "omni_base_controller_prefix":  "/omni_base_controller",
            "plan_with_constraints_service":"/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service":  "/plan_with_hand_line",
            "plan_with_joint_goals_service":"/plan_with_joint_goals",
            "timeout":                       30.0,
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ],
            "rgbd_sensor_frame": "head_rgbd_sensor_link",
            "passive_joints": [
                "hand_r_spring_proximal_joint",
                "hand_l_spring_proximal_joint"
            ],
            "looking_hand_constraint": {
                "plugin_name": "hsrb_planner_plugins/LookHand",
                "use_joints": ["head_pan_joint", "head_tilt_joint"]
            },
            "motion_planning_joints": [
                "wrist_flex_joint",
                "wrist_roll_joint",
                "arm_roll_joint",
                "arm_flex_joint",
                "arm_lift_joint",
                "hand_motor_joint",
                "head_pan_joint",
                "head_tilt_joint"
            ]
        }
    },
    "end_effector": {
        "gripper": {
            "class":        ["end_effector", "Gripper"],
            "joint_names":  ["hand_motor_joint"],
            "prefix":       "/gripper_controller",
            "left_finger_joint_name":  "hand_l_spring_proximal_joint",
            "right_finger_joint_name": "hand_r_spring_proximal_joint"
        }
    },
    "mobile_base": {
        "omni_base": {
            "class": ["mobile_base", "MobileBase"],
            "navigation_action":               "/navigate_to_pose",
            "follow_path_action":               "/follow_path",
            "follow_trajectory_action":  "/omni_base_controller",
            "pose_topic":                "/global_pose",
            "goal_topic":                "/base_goal",
            "timeout":                   1.0
        }
    }
}
"""

_SETTINGS = json.loads(_HSRB_SETTINGS)


def get_entry_by_name(name):
    """Get a resource configuration by `name` from a robot setting dictionary.

    Args:
        name (str): A target resource name.

    Returns:


    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError: No such resource.
    """
    for section, entries in _SETTINGS.items():
        for key, config in entries.items():
            if name == key:
                return section, config
    msg = "Item {0} is not found".format(name)
    raise exceptions.ResourceNotFoundError(msg)


def get_section(section):
    """Get a `section` from a robot setting dictionary.

    Returns:
        Dict[str, JSON Data]: A section data.
    """
    return _SETTINGS.get(section, None)


def get_entry(section, name):
    """Get an entry in robot setting dictionary.

    Args:
        section (str): A section name.
        name (str): A resource name.

    Returns:
        Dict[str, JSON Data]: A corresponding settings.

    Raises:
        hsrb_interface.exceptions.ResourceNotFoundError:
            A resource which has name `name` does not exist.
    """
    if section in _SETTINGS:
        result = _SETTINGS[section].get(name, None)
        if result is None:
            msg = "{0}({1}) is not found".format(section, name)
            raise exceptions.ResourceNotFoundError()
        else:
            return result
    else:
        msg = "{0}({1}) is not found".format(section, name)
        raise exceptions.ResourceNotFoundError(msg)


def get_frame(name):
    """Get an acutal frame id from user-friendly `name`.

    Args:
        name (str): Target frame name.

    Returns:
        str: An actual frame id.
    """
    return get_entry('frame', name)['frame_id']
