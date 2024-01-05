# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.joint_group module"""
from __future__ import absolute_import

import math
import os
import sys
from unittest.mock import call, patch, PropertyMock

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped

import _testing as testing
from hsrb_interface import geometry
from hsrb_interface import joint_group
from hsrb_interface import settings

from hsrb_interface.joint_group import JointGroup

from nose.tools import assert_almost_equal
# from nose.tools import assert_false
# from nose.tools import assert_in
# from nose.tools import assert_not_in
from nose.tools import assert_true
from nose.tools import eq_
from nose.tools import raises

from rclpy.duration import Duration
from sensor_msgs.msg import JointState
import xacro


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


class WholeBodyTest(testing.RosMockTestCase):

    def setUp(self):
        super(WholeBodyTest, self).setUp()

        # patcher = patch("hsrb_interface.trajectory.TrajectoryController")
        # self.traj_controller_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        # patcher = patch("hsrb_interface.trajectory.ImpedanceController")
        # self.imp_controller_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        # patcher = patch("hsrb_interface.trajectory.wait_controllers")
        # self.wait_controllers_mock = patcher.start()
        # self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.robot._get_tf2_buffer")
        self.get_tf2_buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)

        patcher = patch("hsrb_interface.robot._get_tf2_buffer")
        self.get_tf2_buffer_mock = patcher.start()
        self.addCleanup(patcher.stop)

        self.tf2_buffer_mock = self.get_tf2_buffer_mock.return_value

        patcher = patch(
            "hsrb_interface.robot_model.RobotModel.from_parameter_server")
        self.addCleanup(patcher.stop)
        xacro_file = xacro.process_file(
            os.path.join(
                get_package_share_directory('hsrb_description'),
                'robots',
                'hsrb4s.urdf.xacro'
            )
        )
        model = xacro_file.toprettyxml(indent='  ')
        patcher = patch("hsrb_interface.utils.CachingSubscriber")
        self.caching_sub_mock = patcher.start()
        self.addCleanup(patcher.stop)
        caching_sub_mock = self.caching_sub_mock.return_value
        data_mock = PropertyMock(return_value=self.joint_state_fixture())
        # PropertyMock must be attached to class
        type(caching_sub_mock).data = data_mock

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
            "joint_states_topic": "/hsrb/joint_states",
            "arm_controller_prefix": "/hsrb/arm_trajectory_controller",
            "head_controller_prefix": "/hsrb/head_trajectory_controller",
            "hand_controller_prefix": "/hsrb/gripper_controller",
            "omni_base_controller_prefix": "/hsrb/omni_base_controller",
            "plan_with_constraints_service": "/plan_with_constraints",
            "plan_with_hand_goals_service": "/plan_with_hand_goals",
            "plan_with_hand_line_service": "/plan_with_hand_line",
            "plan_with_joint_goals_service": "/plan_with_joint_goals",
            "timeout": 1.0,
            "end_effector_frames": [
                "hand_palm_link",
                "hand_l_finger_vacuum_frame"
            ],
            "rgbd_sensor_frame": "rgbd_sensor_frame",
            "passive_joints": [
                "hand_r_spring_proximal_joint",
                "hand_l_spring_proximal_joint"
            ],
            "looking_hand_constraint": {
                "plugin_name": "LookHand",
                "use_joints": ["head_joint"]
            },
            "motion_planning_joints": [
                "wrist_flex_joint",
                "wrist_roll_joint",
                "arm_roll_joint",
                "arm_flex_joint",
                "arm_lift_joint"
            ]
        }
        self.trajectory_setting = {
            "impedance_control": "/hsrb/impedance_control",
            "constraints_filter_service":
                "/trajectory_filter/filter_trajectory_with_constraints",
            "timeopt_filter_service": "/hsrb/omni_base_timeopt_filter",
            "filter_timeout": 30.0,
            "action_timeout": 30.0,
            "watch_rate": 30.0
        }

    def joint_state_fixture(self):
        joint_state = JointState()
        joint_state.header.stamp.sec = 1755
        joint_state.header.stamp.nanosec = 742000000
        joint_state.header.frame_id = ''
        joint_state.name = ['arm_flex_joint',
                            'arm_lift_joint',
                            'arm_roll_joint',
                            'base_l_drive_wheel_joint',
                            'base_r_drive_wheel_joint',
                            'base_roll_joint',
                            'hand_l_spring_proximal_joint',
                            'hand_motor_joint',
                            'hand_r_spring_proximal_joint',
                            'head_pan_joint',
                            'head_tilt_joint',
                            'wrist_flex_joint',
                            'wrist_roll_joint']
        joint_state.position = [-1.205073719123817e-05,
                                -2.353910232189444e-06,
                                6.731640830537344e-06,
                                0.00015132648675386662,
                                0.06711735857956125,
                                -1.6177818213947148e-06,
                                5.231850970766061e-05,
                                -4.0684104896548945e-05,
                                -7.597231270750626e-06,
                                5.737535211380873e-07,
                                0.0010059283317085388,
                                -0.0003140775579737465,
                                -1.972350262580136e-05]
        joint_state.velocity = [-4.398963494311168e-05,
                                0.0008855797998572822,
                                0.005013974899631351,
                                0.005780376305309976,
                                -0.0006153048272357485,
                                -0.0009519281689300109,
                                0.010811523126418292,
                                0.0015249207867800074,
                                0.0006157871810664606,
                                -0.001243430418769777,
                                0.0009314696615672182,
                                -0.0018553772677545174,
                                -0.005630832679255584]
        joint_state.effort = [0.537512390465002,
                              1.2225493661258626,
                              0.02403405582857048,
                              0.15090609010962908,
                              -0.06012715388461867,
                              -0.35157021739766886,
                              0.0,
                              0.0,
                              0.0,
                              -0.0026258098408504793,
                              -1.2449754755678555,
                              0.2814905678629742,
                              -0.017750577272845902]
        return joint_state

    def initial_tf_fixtures(self):
        odom_to_robot_transform = TransformStamped()
        odom_to_robot_transform.header.stamp.secs = 130
        odom_to_robot_transform.header.stamp.nsecs = 422000000
        odom_to_robot_transform.header.frame_id = 'odom'
        odom_to_robot_transform.child_frame_id = 'base_footprint'
        odom_to_robot_transform.transform.translation.x = 0.0
        odom_to_robot_transform.transform.translation.y = 0.0
        odom_to_robot_transform.transform.translation.z = 0.0
        odom_to_robot_transform.transform.rotation.x = 0
        odom_to_robot_transform.transform.rotation.y = 0
        odom_to_robot_transform.transform.rotation.z = 0
        odom_to_robot_transform.transform.rotation.w = 1

        odom_to_hand_transform = TransformStamped()
        odom_to_hand_transform.header.stamp.secs = 117
        odom_to_hand_transform.header.stamp.nsecs = 900000000
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

    def test_creation(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        JointGroup('whole_body')
        self.get_entry_mock.assert_has_calls([
            call('joint_group', 'whole_body'),
            # call('trajectory', 'impedance_control'),
        ])
        # self.traj_controller_mock.assert_has_calls([
        #     call(self.joint_group_setting['arm_controller_prefix']),
        #     call(self.joint_group_setting['head_controller_prefix']),
        #     call(self.joint_group_setting['hand_controller_prefix']),
        #     call(self.joint_group_setting['omni_base_controller_prefix'],
        #          '/base_coordinates'),
        # ])
        # self.imp_controller_mock.assert_called_with(self.trajectory_setting)

    def test_end_effector_frame_ok(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        whole_body = JointGroup('whole_body')
        eq_('hand_palm_link', whole_body.end_effector_frame)
        frames = ('hand_palm_link', 'hand_l_finger_vacuum_frame')
        eq_(frames, whole_body.end_effector_frames)
        whole_body.end_effector_frame = 'hand_l_finger_vacuum_frame'
        eq_('hand_l_finger_vacuum_frame', whole_body.end_effector_frame)

    @raises(ValueError)
    def test_end_effector_frame_ng(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        whole_body.end_effector_frame = 'hoge'

    def test_joint_state(self):
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]
        whole_body = JointGroup('whole_body')
        eq_(self.joint_state_fixture(), whole_body.joint_state)

    def test_get_end_effector_pose(self):
        # Setup pre-conditions
        self.get_entry_mock.side_effect = [
            self.joint_group_setting,
            # self.trajectory_setting,
        ]

        whole_body = JointGroup('whole_body')
        whole_body.get_end_effector_pose()

        # Check post-conditions
        self.tf2_buffer_mock.lookup_transform.assert_called_with(
            settings.get_frame('base'),
            settings.get_frame('hand'),
            self.connection_mock.get_clock().now(),
            Duration(seconds=5.0)
        )

    def test_inverse_pose(self):
        pose = geometry.pose(1, 2, 3)
        inv = joint_group._invert_pose(pose)
        assert_true(isinstance(inv.pos, geometry.Vector3))
        assert_true(isinstance(inv.ori, geometry.Quaternion))
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

        pose = geometry.pose(ej=math.pi / 4.0)
        inv = joint_group._invert_pose(pose)
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

        pose = geometry.pose(1, 3, 5,
                             ei=math.pi / 3.0,
                             ej=math.pi / 4.0,
                             ek=-math.pi / 8)
        inv = joint_group._invert_pose(pose)
        result = geometry.multiply_tuples(inv, pose)
        assert_almost_equal(result[0][0], 0)
        assert_almost_equal(result[0][1], 0)
        assert_almost_equal(result[0][2], 0)
        assert_almost_equal(result[1][0], 0)
        assert_almost_equal(result[1][1], 0)
        assert_almost_equal(result[1][2], 0)
        assert_almost_equal(result[1][3], 1)

    def test_movement_axis_and_distance(self):
        pose1 = geometry.pose(0, 0, 0)
        pose2 = geometry.pose(2, 0, 0)
        axis, distance = joint_group._movement_axis_and_distance(pose1, pose2)
        assert_almost_equal(axis[0], 1)
        assert_almost_equal(axis[1], 0)
        assert_almost_equal(axis[2], 0)
        assert_almost_equal(distance, 2)
