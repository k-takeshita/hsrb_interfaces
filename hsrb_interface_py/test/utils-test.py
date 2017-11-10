#!/usr/bin/env python
# vim: fileencoding=utf-8
# Copyright (C) 2016 Toyota Motor Corporation
"""Unittest for hsrb_interface.utils module."""

from hsrb_interface import utils
from nose.tools import eq_


def test_iterate():
    """Test iterate function."""
    data = list(utils.iterate(lambda: 2, 5))
    eq_(len(data), 5)
    for datum in data:
        eq_(2, datum)
