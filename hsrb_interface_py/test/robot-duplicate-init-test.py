#!/usr/bin/env python
# Copyright (C) 2017 Toyota Motor Corporation
"""Unittest intialize hsrb_interface.robot module."""

import unittest
import pytest
import rclpy
from rclpy.node import Node

import hsrb_interface

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
        node = Node('test_node_name')       
        hsrb_interface.Robot()


