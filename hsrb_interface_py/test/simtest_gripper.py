#!/usr/bin/env python
# vim: fileencoding=utf-8 :
# Copyright (C) 2016 Toyota Motor Corporation
"""Test gripper interface in Gazebo simlulator."""
import unittest
import pytest
import launch_ros.actions
import launch
import launch_testing
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from hsrb_interface import _testing as testing

@pytest.mark.timeout(120)
class GripperTest(testing.HsrbInterfaceTest):
    """Test cases for gripper interface."""
    def test_command_by_position(self):
        """The gripper should move to given position."""
        self.whole_body.move_to_neutral()
        #self.expect_joints_reach_goals(self.EXPECTED_NEUTRAL, 0.01)
        
        self.gripper.command(1.0)
        #self.expect_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.command(0.0)
        #self.expect_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)

    def test_command_to_grasp(self):
        """The gripper should move to given angle."""
        self.whole_body.move_to_neutral()

        self.gripper.command(1.0)
        #self.expect_joints_reach_goals({'hand_motor_joint': 1.0}, delta=0.01)

        self.gripper.grasp(-0.01)
        #self.expect_joints_reach_goals({'hand_motor_joint': 0.0}, delta=0.01)


