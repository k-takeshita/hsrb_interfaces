#!/usr/bin/env python
# vim: fileencoding=utf-8

import math

from unittest import TestCase
from nose.tools import ok_, eq_
import mock


from hsrb_interface import geometry

def test_shortest_angular_distance():
    eq_(math.pi, geometry.shortest_angular_distance(0.0, math.pi))
    eq_(math.pi / 2.0, geometry.shortest_angular_distance(0.0, math.pi / 2.0))
    eq_(-math.pi / 2.0, geometry.shortest_angular_distance(0.0, math.pi / 2.0 * 3.0))
