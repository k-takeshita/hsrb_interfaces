#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""

import unittest

import hsrb_interface
import pytest
import rclpy


@pytest.mark.launch_test
class DuplicateInitTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_duplicate_init(self):
        """Test duplicate init

        only check no throw.
        """
        hsrb_interface.Robot()
