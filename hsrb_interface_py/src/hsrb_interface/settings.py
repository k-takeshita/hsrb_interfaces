#!/usr/bin/env python
# vim: fileencoding=utf-8
u"""ROS Graph 名のマッピングテーブル

仕様変更を吸収するための名前対応付けを行うテーブル。
今は単純な名前のルックアップのみのため、複雑な仕様変更には耐えられない。
今後大きな仕様変更がありうるので注意。
"""


hsrb = {
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

hsr_beetle = {
    'mobile_base_names':              ['mobile_base'],
    'joint_group_names':              ['whole_body'],
    'map_frame_id':                   "floor/reproduce_ant_task",
    'odom_frame_id':                  "odom",
    'base_frame_id':                  "base_footprint",
    'hand_frame_id':                  "gripper_palm_link",
    'text_to_speech_topic':           "/talk_request",
    'text_to_speech_topic':           "/talk_request",
    'goal_topic':                     "/base_goal",
    'pose_topic':                     "/laser_2d_pose_ref",
    'laser_topic':                    "/scan",
    'imu_topic':                      "/imu/data",
    'wrist_wrench_topic':             "/wrench_state",
    'joint_states_topic':             "/joint_states",
    'recognized_object_topic':        "/recognized_object",
    'arm_controller_prefix':          "/hsr_beetle/arm_controller",
    'head_controller_prefix':         "/hsr_beetle/head_controller",
    'hand_controller_prefix':         "/hsr_beetle/gripper_controller",
    'hand_joint_names':               ["motor_proximal_joint"],
    'omni_base_controller_prefix':    "/hsr_beetle/omni_base_controller",
    'head_l_stereo_camera_prefix':    "/stereo_camera/left",
    'head_r_stereo_camera_prefix':    "/stereo_camera/right",
    'head_center_camera_prefix':      "/",
    'hand_camera_prefix':             "/",
    'rgbd_sensor_prefix':             "/camera",
    'move_base_action':               "/move_base/move",
    'battery_state':                  "/battery_state",
    'suction_controller':             "/suction_control",
    'suction_topic':                  "/suction_on",
    'pressure_sensor_topic':          "/pressure_sensor_on",
    'trajectory_filter_service':      "/trajectory_filter/filter_trajectory_with_constraints",
    'plan_with_constraints_service':  "/plan_with_constraints",
    'plan_with_hand_goals_service':   "/plan_with_hand_goals",
    'plan_with_hand_line_service':    "/plan_with_hand_line",
    'plan_with_joint_goals_service':  "/plan_with_joint_goals",
}

settings = hsr_beetle
