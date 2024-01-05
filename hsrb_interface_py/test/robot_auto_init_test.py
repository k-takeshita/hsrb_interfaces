#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""

import unittest

import hsrb_interface
import pytest

import rclpy


@pytest.mark.launch_test
class AutoInitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_auto_init(self):
        """Test auto init rclpy"""
        """Test auto init rclpy"""
        hsrb_interface.Robot()
        self.assertTrue(rclpy.ok())
