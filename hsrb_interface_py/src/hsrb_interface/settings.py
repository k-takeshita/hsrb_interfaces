#!/usr/bin/env python
# vim: fileencoding=utf-8
u"""ROS Graph 名のマッピングテーブル

仕様変更を吸収するための名前対応付けを行うテーブル。
今は単純な名前のルックアップのみのため、複雑な仕様変更には耐えられない。
今後大きな仕様変更がありうるので注意。
"""

import json
from . import exceptions

VERSION = "1.0.0"


HSRB = """
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
    "joint_group": {
        "whole_body": {
            "class": ["joint_group", "JointGroup"],
            "joint_states_topic":             "/hsrb/joint_states",
            "arm_controller_prefix":          "/hsrb/arm_trajectory_controller",
            "head_controller_prefix":         "/hsrb/head_trajectory_controller",
            "hand_controller_prefix":         "/hsrb/gripper_controller",
            "omni_base_controller_prefix":    "/hsrb/omni_base_controller",
            "trajectory_filter_service":      "/trajectory_filter/filter_trajectory_with_constraints",
            "plan_with_constraints_service":  "/plan_with_constraints",
            "plan_with_hand_goals_service":   "/plan_with_hand_goals",
            "plan_with_hand_line_service":    "/plan_with_hand_line",
            "plan_with_joint_goals_service":  "/plan_with_joint_goals"
        }
    },
    "end_effector": {
        "gripper": {
            "class": ["end_effector", "Gripper"],
            "joint_names":  ["hand_motor_joint"],
            "prefix":       "/hsrb/gripper_controller"
        },
        "suction": {
            "class": ["end_effector", "Suction"],
            "action":                         "/suction_control",
            "suction_topic":                  "/suction_on",
            "pressure_sensor_topic":          "/pressure_sensor"
        }
    },
    "mobile_base": {
        "omni_base": {
            "class": ["mobile_base", "MobileBase"],
            "move_base_action":  "/move_base/move",
            "pose_topic":        "/global_pose",
            "goal_topic":        "/base_goal"
        }
    },
    "camera": {
        "head_l_stereo_camera": {
            "class": ["sensors", "Camera"],
            "prefix": "/hsrb/head_l_stereo_camera"
        },
        "head_r_stereo_camera": {
            "class": ["sensors", "Camera"],
            "prefix":  "/hsrb/head_l_stereo_camera"
        },
        "head_rgbd_sensor_rgb": {
            "class": ["sensors", "Camera"],
            "prefix": "/hsrb/head_rgbd_sensor/rgb/image_raw"
        },
        "head_rgbd_sensor_depth": {
            "class": ["sensors", "Camera"],
            "prefix": "hsrb/head_rgbd_sensor/depth/image_raw"
        }
    },
    "imu": {
        "base_imu": {
            "class": ["sensors", "IMU"],
            "topic": "/hsrb/base_imu/data"
        }
    },
    "force_torque": {
        "wrist_wrench": {
            "class": ["sensors", "ForceTorque"],
            "topic": "/hsrb/wrench_state"
        }
    },
    "lidar": {
        "base_scan": {
            "class": ["sensors", "Lidar"],
            "topic": "/hsrb/base_sscan"
        }
    },
    "object_detection": {
        "marker": {
            "class": ["object_detection", "ObjectDetector"],
            "topic": "/recognized_object"
        }
    },
    "power_supply": {
        "battery": {
            "class": ["battery", "Battery"],
            "topic": "/hsrb/battery_state"
        }
    },
    "text_to_speech": {
        "default": {
            "class": ["text_to_speech", "TextToSpeech"],
            "topic": "/talk_request"
        }
    },
    "collision_world": {
        "default": {
            "class": ["collision_world", "CollisionWorld"],
            "service": "/get_collision_environment"
        }
    }
}
"""


HSR_BEETLE = """
{
    "robot": {
        "hsrb": {
            "fullname": "HSR-Beetle"
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
            "id": "gripper_palm_link"
        }
    },
    "joint_group": {
        "whole_body": {
            "joint_states_topic":             "/joint_states",
            "arm_controller_prefix":          "/hsr_beetle/arm_controller",
            "head_controller_prefix":         "/hsr_beetle/head_controller",
            "hand_controller_prefix":         "/hsr_beetle/gripper_controller",
            "omni_base_controller_prefix":    "/hsr_beetle/omni_base_controller",
            "trajectory_filter_service":      "/trajectory_filter/filter_trajectory_with_constraints",
            "plan_with_constraints_service":  "/plan_with_constraints",
            "plan_with_hand_goals_service":   "/plan_with_hand_goals",
            "plan_with_hand_line_service":    "/plan_with_hand_line",
            "plan_with_joint_goals_service":  "/plan_with_joint_goals"
        }
    },
    "end_effector": {
        "gripper": {
            "joint_names":  ["motor_proximal_joint"],
            "prefix":       "/hsr_beetle/gripper_controller"
        },
        "suction": {
            "action":                         "/suction_control",
            "suction_topic":                  "/suction_on",
            "pressure_sensor_topic":          "/pressure_sensor"
        }
    },
    "mobile_base": {
        "omni_base": {
            "move_base_action":  "/move_base/move",
            "pose_topic":        "/global_pose",
            "goal_topic":        "/base_goal"
        }
    },
    "camera": {
        "head_l_stereo_camera": {
            "prefix": "/stereo_camera/left"
        },
        "head_r_stereo_camera": {
            "prefix":  "/stereo_camera/right"
        },
        "head_rgbd_sensor_rgb": {
            "prefix": "/camera/rgb/image_raw"
        },
        "head_rgbd_sensor_depth": {
            "prefix": "/camera/depth/image_raw"
        },
    },
    "imu": {
        "base_imu": {
            "topic": "/imu/data"
        }
    },
    "force_torque": {
        "wrist_wrench": {
            "topic": "/wrench_state"
        }
    },
    "lidar": {
        "base_scan": {
            "topic": "/scan"
        }
    },
    "object_detection": {
        "marker": {
            "topic": "/recognized_object"
        }
    },
    "power_supply": {
        "battery": {
            "topic": "/battery_state"
        }
    },
    "text_to_speech": {
        "default": {
            "topic": "/talk_request"
        }
    },
    "collision_world": {
        "default": {
            "service": "/get_collision_environment"
        }
    }
}
"""

_settings = json.loads(HSRB)

def get_section(section):
    return _settings.get(section, None)


def get_entry(section, name):
    if section in _settings:
        result = _settings[section].get(name, None)
        if result is None:
            raise exceptions.ResourceNotFoundError("{0}({1}) is not found".format(section, name))
        else:
            return result
    else:
        raise exceptions.ResourceNotFoundError("{0}({1}) is not found".format(section, name))

def get_frame(name):
    return get_entry('frame', name)['frame_id']
