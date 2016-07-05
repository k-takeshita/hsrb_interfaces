#!/usr/bin/env python
# vim: fileencoding=utf-8
"""Unittest for hsrb_interface.geometry module"""

import math

from nose.tools import eq_

from hsrb_interface import geometry


def test_shortest_angular_distance():
    """Test geometry.shortest_angular_distance function"""
    eq_(math.pi, geometry.shortest_angular_distance(0.0, math.pi))
    eq_(math.pi / 2.0, geometry.shortest_angular_distance(0.0, math.pi / 2.0))
    eq_(-math.pi / 2.0,
        geometry.shortest_angular_distance(0.0, math.pi / 2.0 * 3.0))
