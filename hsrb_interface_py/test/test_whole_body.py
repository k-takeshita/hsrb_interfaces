# Copyright (C) 2024 Toyota Motor Corporation
from __future__ import absolute_import

import math
import os
import sys

from unittest.mock import patch

import _testing as testing
from geometry_msgs.msg import TransformStamped
from hsrb_interface import geometry
from hsrb_interface.joint_group import JointGroup

from nose.tools import eq_
from nose.tools import raises
import rospkg

from tmc_planning_msgs.msg import TaskSpaceRegion

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

_MANIPULATION_AREA_SIZE = 100.0


class WholeBodyTest(testing.RosMockTestCase):

    def setUp(self):
        super(WholeBodyTest, self).setUp()

        patcher = patch("hsrb_interface.trajectory.TrajectoryController")
        self.traj_controller_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.robot._get_tf2_buffer")
        self.get_tf2_buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)
        self.tf2_buffer_mock = self.get_tf2_buffer_mock.return_value

        rp = rospkg.RosPack()
        urdf_xml = os.path.join(rp.get_path('hsrb_description'),
                                'robots', 'hsrb4s.urdf.xacro')
        with open(urdf_xml) as f:
            model = [f.read()]
        self.get_param_mock.return_value = model

        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.joint_group.KinematicsInterface")
        self.kinematics_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("rclpy.spin_until_future_complete")
        self.spin_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("asyncio.run")
        self.async_run_mock = patcher.start()
        self.addCleanup(patcher.stop)

        def get_frame_side_effect(key):
            mapping = {
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
            }
            return mapping[key]["frame_id"]
        self.get_frame_mock.side_effect = get_frame_side_effect

        self.joint_group_setting = {
            "class": ["joint_group", "JointGroup"],
            "joint_states_topic": "/joint_states",
            "arm_controller_prefix": "/arm_trajectory_controller",
            "head_controller_prefix": "/head_trajectory_controller",
            "hand_controller_prefix": "/gripper_controller",
            "omni_base_controller_prefix": "/omni_base_controller",
            "plan_with_constraints_service": "/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service": "/plan_with_hand_line",
            "plan_with_joint_goals_service": "/plan_with_joint_goals",
            "timeout": 30.0,
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

        self.trajectory_setting = {
            "impedance_control": "/hsrb/impedance_control",
            "constraint_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "whole_timeopt_filter_service": "/timeopt_filter_node/filter_trajectory",
            "caster_joint": "base_roll_joint",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
        }

    def initial_tf_fixtures(self):
        odom_to_robot_transform = TransformStamped()
        odom_to_robot_transform.header.stamp.sec = 130
        odom_to_robot_transform.header.stamp.nanosec = 422000000
        odom_to_robot_transform.header.frame_id = 'odom'
        odom_to_robot_transform.child_frame_id = 'base_footprint'
        odom_to_robot_transform.transform.translation.x = 0.0
        odom_to_robot_transform.transform.translation.y = 0.0
        odom_to_robot_transform.transform.translation.z = 0.0
        odom_to_robot_transform.transform.rotation.x = 0.0
        odom_to_robot_transform.transform.rotation.y = 0.0
        odom_to_robot_transform.transform.rotation.z = 0.0
        odom_to_robot_transform.transform.rotation.w = 1.0

        odom_to_hand_transform = TransformStamped()
        odom_to_hand_transform.header.stamp.sec = 117
        odom_to_hand_transform.header.stamp.nanosec = 900000000
        odom_to_hand_transform.header.frame_id = 'odom'
        odom_to_hand_transform.child_frame_id = 'hand_palm_link'
        odom_to_hand_transform.transform.translation.x = 0.158027351797
        odom_to_hand_transform.transform.translation.y = 0.0785128775482
        odom_to_hand_transform.transform.translation.z = 0.825494221876
        odom_to_hand_transform.transform.rotation.x = 0.000202774340021
        odom_to_hand_transform.transform.rotation.y = 1.89798372121e-07
        odom_to_hand_transform.transform.rotation.z = 0.999999526031
        odom_to_hand_transform.transform.rotation.w = -0.000952270991235

        return odom_to_robot_transform, odom_to_hand_transform

    def test_constraint_tsrs(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        odom_to_robot_transform, odom_to_hand_transform = self.initial_tf_fixtures()
        self.async_run_mock.side_effect = [
            odom_to_robot_transform,
            odom_to_hand_transform,
        ]
        whole_body = JointGroup('whole_body')

        constraint_tsr = TaskSpaceRegion()
        constraint_tsr.end_frame_id = whole_body.end_effector_frame
        constraint_tsr.origin_to_tsr.orientation.w = 1.0
        constraint_tsr.tsr_to_end = geometry.tuples_to_pose(
            whole_body.get_end_effector_pose('odom')
        )
        constraint_tsr.min_bounds[0] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[1] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[2] = -_MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[0] = _MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[1] = _MANIPULATION_AREA_SIZE
        constraint_tsr.max_bounds[2] = _MANIPULATION_AREA_SIZE
        constraint_tsr.min_bounds[3] = -0.3
        constraint_tsr.min_bounds[4] = -0.3
        constraint_tsr.max_bounds[3] = 0.3
        constraint_tsr.max_bounds[4] = 0.3
        constraint_tsr.min_bounds[5] = -math.pi
        constraint_tsr.max_bounds[5] = math.pi

        whole_body.constraint_tsrs = [constraint_tsr]

        eq_(constraint_tsr, whole_body.constraint_tsrs[0])

    @raises(ValueError)
    def test_constraint_tsrs_not_list(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.constraint_tsrs = "hoge"

    @raises(TypeError)
    def test_constraint_tsrs_not_tsr(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.constraint_tsrs = ["hoge"]
