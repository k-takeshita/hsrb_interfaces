#!/usr/bin/env python
# vim: fileencoding=utf-8
u"""ROS Graph 名のマッピングテーブル

仕様変更を吸収するための名前対応付けを行うテーブル。
今は単純な名前のルックアップのみのため、複雑な仕様変更には耐えられない。
今後大きな仕様変更がありうるので注意。
"""

from .exceptions import ResourceNotFoundError


HSRB = {
    'map_frame_id':                   "floor/reproduce_ant_task",
    'odom_frame_id':                  "odom",
    'base_frame_id':                  "base_footprint",
    'hand_frame_id':                  "hand_palm_link",
    'text_to_speech_topic':           "/talk_request",
    'goal_topic':                     "/goal",
    'pose_topic':                     "/laser_2d_pose_ref",
    'laser_topic':                    "/hsrb/base_scan",
    'imu_topic':                      "/hsrb/base_imu",
    'wrist_wrench_topic':             "/hsrb/wrist_wrench",
    'joint_states_topic':             "/hsrb/joint_states",
    'recognized_object_topic':        "/hsrb/ecognized_object",
    'arm_controller_prefix':          "/hsrb/arm_trajectory_controller",
    'head_controller_prefix':         "/hsrb/head_trajectory_controller",
    'hand_controller_prefix':         "/hsrb/gripper_controller",
    'hand_joint_names':               ["hand_motor_joint"],
    'head_controller_prefix':         "/hsrb/head_trajectory_controller",
    'omni_base_controller_prefix':    "/hsrb/omni_base_controller",
    'head_l_stereo_camera_prefix':    "/hsrb/head_l_stereo_camera",
    'head_r_stereo_camera_prefix':    "/hsrb/head_r_stereo_camera",
    'head_center_camera_prefix':      "/hsrb/head_center_camera",
    'hand_camera_prefix':             "/hsrb/hand_camera",
    'rgbd_sensor_prefix':             "/hsrb/rgbd_sensor",
    'move_base_action':               "/move_base/move",
    'battery_state':                  "/hsrb/battery_state",
    'suction_controller':             "/hsrb/sucction_controller",
    'pressure_sensor_topic':          "/sucction_on",
    'trajectory_filter_service':      "/pressure_sensor_on",
    'trajectory_filter_service':      "/trajectory_filter/filter_trajectory_with_constraints",
    'plan_with_constraints_service':  "/plan_with_constraints",
    'plan_with_hand_goals_service':   "/plan_with_hand_goals",
    'plan_with_hand_line_service':    "/plan_with_hand_line",
    'plan_with_joint_goals_service':  "/plan_with_joint_goals",
}

HSR_BEETLE = {
    'robot': {
        'name': 'hsrb',
    },
    'frame': {
        'map':  "floor/reproduce_ant_task",
        'odom': "odom",
        'base': "base_footprint",
        'hand': "gripper_palm_link",
    },
    'joint_group': [
        {
            'name':                           'whole_body',
            'joint_states_topic':             "/joint_states",
            'arm_controller_prefix':          "/hsr_beetle/arm_controller",
            'head_controller_prefix':         "/hsr_beetle/head_controller",
            'hand_controller_prefix':         "/hsr_beetle/gripper_controller",
            'omni_base_controller_prefix':    "/hsr_beetle/omni_base_controller",
            'trajectory_filter_service':      "/trajectory_filter/filter_trajectory_with_constraints",
            'plan_with_constraints_service':  "/plan_with_constraints",
            'plan_with_hand_goals_service':   "/plan_with_hand_goals",
            'plan_with_hand_line_service':    "/plan_with_hand_line",
            'plan_with_joint_goals_service':  "/plan_with_joint_goals",
        },
    ],
    'end_effector': [
        {
            'name':         'gripper',
            'joint_names':  ["motor_proximal_joint"],
            'prefix':       "/hsr_beetle/gripper_controller",
        },
        {
            'name':                           'suction',
            'action':                         "/suction_control",
            'suction_topic':                  "/suction_on",
            'pressure_sensor_topic':          "/pressure_sensor",
        },
    ],
    'mobile_base': [
        {
            'name':              'omni_base',
            'move_base_action':  "/move_base/move",
            'pose_topic':        "/laser_2d_pose_ref",
            'goal_topic':        "/base_goal",
        },
    ],
    'camera': [
        {
            'name':   'head_l_stereo_camera',
            'prefix': "/stereo_camera/left"
        },
        {
            'name':    'head_r_stereo_camera',
            'prefix':  "/stereo_camera/right"
        },
        {
            'name':    'head_rgbd_sensor_rgb',
            'prefix': '/camera/rgb'
        },
        {
            'name': 'head_rgbd_sensor_depth',
            'prefix': '/camera/depth'
        },
    ],
    'imu': [
        {
            'name': 'base_imu',
            'topic': "/imu/data"
        }
    ],
    'force_torque': [
        {
            'name':  'wrist_wrench',
            'topic': "/wrench_state"
        }
    ],
    'laser_scan': [
        {
            'name': 'base_scan',
            'topic': "/scan",
        }
    ],
    'object_detector': [
        {
            'name':  'marker',
            'topic': "/recognized_object",
        }
    ],
    'power_source': [
        {
            'name': 'battery',
            'topic': "/battery_state"
        }
    ],
    'text_to_speech': [
        {
            'name': 'default',
            'topic': '/talk_request',
        }
    ],
    'collision_map': [
        {
            'name': 'default',
            'service': '/get_collision_environment'
        }
    ],
}

settings = HSR_BEETLE

def get_setting(section, name):
    section = settings[section]
    for entry in section:
        if entry['name'] == name:
            return entry
    else:
        raise ResourceNotFoundError('Resource {0} not found'.format(name))

def get_frame(name):
    return settings['frame'][name]

