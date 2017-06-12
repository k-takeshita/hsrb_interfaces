# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest for hsrb_interface._extension.KinematicsInterface module"""
import os
import unittest

from hsrb_interface._extension import KinematicsInterface

from nose.tools import assert_almost_equals
from nose.tools import assert_raises
from nose.tools import assert_true
from nose.tools import eq_

import rospkg


class KinematicsInterfaceTest(unittest.TestCase):

    def setUp(self):
        rp = rospkg.RosPack()
        urdf_xml = os.path.join(rp.get_path('hsrb_description'),
                                'robots', 'hsrb3s.urdf')
        f = open(urdf_xml)
        self._kinematics = KinematicsInterface(f.read())

    def test_normal(self):
        """Test normal case"""
        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_rgbd_sensor_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], -0.06196844)
        assert_almost_equals(result['head_tilt_joint'], -0.82959372)

        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_l_stereo_camera_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], -0.07021305)
        assert_almost_equals(result['head_tilt_joint'], -0.70391284)

        result = self._kinematics.calculate_gazing_angles(
            [1.0, 0.0, 0.0], 'head_r_stereo_camera_link')
        eq_(len(result), 2)
        assert_true('head_pan_joint' in result.keys())
        assert_true('head_tilt_joint' in result.keys())
        assert_almost_equals(result['head_pan_joint'], 0.07021305)
        assert_almost_equals(result['head_tilt_joint'], -0.70391284)

    def test_invalid_point_length(self):
        """The dimension of point must be three."""
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0], 'head_rgbd_sensor_link')
        assert_raises(ValueError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0, 0.0, 0.0], 'head_rgbd_sensor_link')

    def test_uncomputable_point(self):
        """If the solution is nothing, return empty dictionary."""
        result = self._kinematics.calculate_gazing_angles(
            [0.0, 0.0, 0.757], 'head_rgbd_sensor_link')
        eq_(len(result), 0)

    def test_invalid_frame(self):
        """A frame name which is not in robot model is not accepted."""
        assert_raises(RuntimeError, self._kinematics.calculate_gazing_angles,
                      [1.0, 0.0, 0.0], 'head_rgbd_sensor_frame')