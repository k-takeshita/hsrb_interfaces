#!/usr/bin/env python
# vim: fileencoding=utf-8

import math

from unittest import TestCase
from nose.tools import ok_, eq_
import mock

from hsrb_interface import utils


def test_iterate():
    xs = list(utils.iterate(lambda: 2,  5))
    eq_(len(xs), 5)
    for x in xs:
        eq_(2, x)
